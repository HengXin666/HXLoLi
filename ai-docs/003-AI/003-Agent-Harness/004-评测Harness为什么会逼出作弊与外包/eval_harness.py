import json, os, subprocess, uuid
from openai import OpenAI

MODEL = os.environ.get("MODEL", "gpt-5")
client = OpenAI()  # key 只在宿主机, 不进容器
TOOLS = [
    {"type": "function", "function": {"name": "bash", "description": "run a shell command in the task container",
        "parameters": {"type": "object", "properties": {"cmd": {"type": "string"}}, "required": ["cmd"]}}},
    {"type": "function", "function": {"name": "submit", "description": "submit when the task is done",
        "parameters": {"type": "object", "properties": {"answer": {"type": "string"}}, "required": ["answer"]}}},
]
CONTINUE = "If you are finished, call submit()."

def sh(box, cmd, timeout=120):
    r = subprocess.run(["docker", "exec", box, "bash", "-lc", cmd], capture_output=True, text=True, timeout=timeout)
    return (r.stdout + r.stderr)[-8000:]

def new_box(image, workdir):
    name = "eval-" + uuid.uuid4().hex[:8]
    subprocess.run(["docker", "run", "-d", "--rm", "--network", "none", "--name", name,
                    "-v", f"{workdir}:/work:ro", image, "sleep", "infinity"], check=True)
    sh(name, "cp -r /work /task")
    return name

def score(box, test_cmd, tests_src):
    sh(box, f"rm -rf /task/tests && cp -r {tests_src} /task/tests")  # 评分前还原测试
    return sh(box, f"cd /task && {test_cmd}; echo EXIT=$?").rstrip().endswith("EXIT=0")

def run(task, image, workdir, test_cmd, attempts=3, n=4, max_msgs=80):
    for k in range(n):  # best-of-N: 每条轨迹一个新容器
        box = new_box(image, workdir)
        msgs = [{"role": "system", "content": "You are an autonomous engineer. Work in /task."},
                {"role": "user", "content": task}]
        tries = 0
        try:
            while len(msgs) < max_msgs:
                m = client.chat.completions.create(model=MODEL, messages=msgs, tools=TOOLS).choices[0].message
                msgs.append(m.model_dump(exclude_none=True))
                if not m.tool_calls:
                    msgs.append({"role": "user", "content": CONTINUE})
                    continue
                for c in m.tool_calls:
                    args = json.loads(c.function.arguments or "{}")
                    if c.function.name == "bash":
                        out = sh(box, args.get("cmd", ""))
                    elif score(box, test_cmd, "/work/tests"):
                        print(f"traj {k}: PASS after {tries + 1} submit(s)")
                        return True
                    else:
                        tries += 1
                        if tries >= attempts:
                            raise StopIteration
                        out = "Your submission was incorrect. Keep working."
                    msgs.append({"role": "tool", "tool_call_id": c.id, "content": out})
        except StopIteration:
            pass
        finally:
            subprocess.run(["docker", "rm", "-f", box], capture_output=True)
        print(f"traj {k}: FAIL")
    return False

if __name__ == "__main__":
    run(task=open("task.md").read(), image="python:3.12", workdir=os.path.abspath("repo"), test_cmd="python -m pytest -q tests")
