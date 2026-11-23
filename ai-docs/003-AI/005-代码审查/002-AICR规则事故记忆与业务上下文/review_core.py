"""AI CR 编排原型：实际规划义务，模型判断由外部端口提供。"""
from __future__ import annotations

from copy import deepcopy
from typing import Any

Record = dict[str, Any]


def expert_rules() -> list[Record]:
    specs = [
        ("GRANT", "auth", "grant_revoke", "撤销旧授权是否误删并发新授权", "grant_contract", "regression"),
        ("RETENTION", "lifecycle", "creator_delete", "删除 creator 是否销毁必须保留的共享资源", "retention_contract", "model"),
        ("REPAIR", "repair", "retry_change", "等待超时是否被错误实现为取消后台恢复", "repair_contract", "model"),
    ]
    return [{
        "id": key, "revision": 1, "state": "active", "repo": "acme/service",
        "prefix": f"src/{area}/", "operation": operation, "critical": True,
        "question": question, "required": ["code", evidence], "cost": 2,
        "checker": checker, "owner": area, "synthetic": False,
    } for key, area, operation, question, evidence, checker in specs]


def library() -> list[Record]:
    rules = expert_rules()
    for index in range(1197):
        auth = index < 20
        rules.append({
            "id": f"SYN-{index:04d}", "revision": 1, "state": "active",
            "repo": "acme/service", "prefix": "src/auth/" if auth else f"src/feature/{index % 60}/",
            "operation": "grant_revoke" if auth else "feature_change",
            "critical": auth, "question": f"合成规则 {index} 的语义检查义务",
            "required": ["code"], "cost": 1, "checker": "model",
            "owner": "fixture", "synthetic": True,
        })
    return rules


def change(case: str) -> Record:
    area, operation = {
        "grant": ("auth", "grant_revoke"),
        "retention": ("lifecycle", "creator_delete"),
        "repair": ("repair", "retry_change"),
    }[case]
    return {"repo": "acme/service", "base": "fixture-base", "head": "fixture-head",
            "paths": [f"src/{area}/service.py"], "operations": [operation],
            "model_issues": [], "principal": "reviewer", "knowledge_version": 3}


def applies(rule: Record, envelope: Record) -> bool:
    return (rule["state"] == "active" and rule["repo"] == envelope["repo"]
            and rule["operation"] in envelope["operations"]
            and any(path.startswith(rule["prefix"]) for path in envelope["paths"]))


def plan(rules: list[Record], envelope: Record) -> list[Record]:
    # 模型未提出 issue 也规划专家问题；真实影响映射在此端口之外。
    return [deepcopy(rule) for rule in rules if applies(rule, envelope)]


def knowledge() -> dict[str, Record]:
    records = {
        "code": ("fixture-head", "当前变更片段由调用者提供；此原型不解析真实仓库。"),
        "grant_contract": ("business-r1", "旧 grant=A 被替换为 B 后，撤销 A 不得删除 B。"),
        "retention_contract": ("business-r2", "软删除关闭 serving；retention 仍要求保留配置，删除 creator 的 cascade 会销毁根。"),
        "repair_contract": ("business-r3", "flush 成功前不得记 repair time；等待者可超时，后台恢复必须继续。"),
    }
    return {key: {"id": key, "revision": revision, "text": text,
                  "state": "active", "acl": ["reviewer"], "valid_until": 99}
            for key, (revision, text) in records.items()}


def evidence_packet(rule: Record, envelope: Record, sources: dict[str, Record]) -> Record:
    evidence, missing = [], []
    for key in rule["required"]:
        item = sources.get(key)
        readable = item and envelope["principal"] in item["acl"]
        current = item and item["state"] == "active" and item["valid_until"] >= envelope["knowledge_version"]
        if not readable or not current:
            missing.append(key)
        else:
            evidence.append(deepcopy(item))
    return {"rule_id": rule["id"], "rule_revision": rule["revision"],
            "question": rule["question"], "code_head": envelope["head"],
            "evidence": evidence, "missing": missing,
            "instruction": "说明专业前提，查违反路径与合法反例；证据不足输出 unknown。"}


def replacement_survives(mode: str) -> bool:
    selected = {"id": "A", "user": "U", "workspace": "W"}
    grants = [{"id": "B", "user": "U", "workspace": "W"}]
    if mode == "identity":
        grants = [grant for grant in grants if grant["id"] != selected["id"]]
    elif mode == "user_workspace":
        grants = [grant for grant in grants if (grant["user"], grant["workspace"]) != ("U", "W")]
    else:
        raise ValueError("未知的回归模式")
    return any(grant["id"] == "B" for grant in grants)


def model_result(packet: Record, verdict: Record | None) -> Record:
    if verdict is None:
        return {"status": "await_model", "reason": "尚未接入模型，不生成虚构判断"}
    if not isinstance(verdict, dict):
        return {"status": "error", "reason": "单条模型结果必须是对象"}
    allowed = {item["id"] for item in packet["evidence"]}
    cited = verdict.get("evidence_ids", [])
    valid_ids = isinstance(cited, list) and all(isinstance(key, str) for key in cited)
    valid_status = verdict.get("status") in {"pass", "violation", "unknown"}
    if not valid_ids or not valid_status or not set(cited) <= allowed:
        return {"status": "error", "reason": "模型结果格式或证据引用无效"}
    if verdict["status"] != "unknown" and not allowed <= set(cited):
        return {"status": "unknown", "reason": "没有引用完整必需证据"}
    return {"status": verdict["status"], "evidence_ids": cited,
            "validation_pending": True, "reason": "仅核验引用，业务推理仍待独立验证"}


def execute(rule: Record, packet: Record, remaining: int, mode: str, verdicts: Record) -> tuple[Record, int]:
    result = {"rule_id": rule["id"], "revision": rule["revision"], "critical": rule["critical"]}
    if remaining < rule["cost"]:
        return result | {"status": "skipped_budget"}, remaining
    remaining -= rule["cost"]
    if packet["missing"]:
        return result | {"status": "unknown", "missing": packet["missing"]}, remaining
    if rule["checker"] == "regression":
        status = "pass" if replacement_survives(mode) else "violation"
        return result | {"status": status, "checker": "minimal_regression"}, remaining
    return result | model_result(packet, verdicts.get(rule["id"])), remaining


def gate(results: list[Record], complete: bool) -> str:
    if any(item["critical"] and item["status"] == "violation" for item in results):
        return "block"
    if not complete or any(item.get("validation_pending") for item in results):
        return "needs_review"
    return "pass"


def run(rules: list[Record], envelope: Record, sources: dict[str, Record], budget: int,
        mode: str = "identity", verdicts: Record | None = None) -> Record:
    applicable = plan(rules, envelope)
    packets = [evidence_packet(rule, envelope, sources) for rule in applicable]
    remaining, results = budget, []
    for rule, packet in zip(applicable, packets):
        result, remaining = execute(rule, packet, remaining, mode, verdicts or {})
        results.append(result)
    critical = [item for item in results if item["critical"]]
    done = sum(item["status"] in {"pass", "violation"} for item in critical)
    complete = bool(applicable) and all(item["status"] in {"pass", "violation"} for item in results)
    return {"library_count": len(rules), "applicable_count": len(applicable),
            "critical_count": len(critical), "critical_executed": done,
            "model_issues_before_planning": len(envelope["model_issues"]),
            "snapshots": {"code": envelope["head"], "policy": "fixture-policy-r1",
                          "knowledge": envelope["knowledge_version"]},
            "budget": budget, "remaining": remaining, "complete": complete,
            "critical_complete": bool(applicable) and done == len(critical),
            "gate": gate(results, complete), "results": results, "packets": packets,
            "scope_note": "精确 repo + 路径前缀 + 显式操作；零义务需补审，影响映射未实现"}


class RuleStore:
    """最小修订历史，源删除变为 retired 而非抹掉历史。"""

    def __init__(self) -> None:
        self.history: dict[str, list[Record]] = {}

    def reconcile(self, source_key: str, content: str | None) -> Record:
        revisions = self.history.setdefault(source_key, [])
        record = {"id": source_key, "revision": len(revisions) + 1,
                  "state": "active" if content is not None else "retired", "content": content}
        revisions.append(record)
        return deepcopy(record)

    def active(self) -> list[Record]:
        return [deepcopy(items[-1]) for items in self.history.values() if items[-1]["state"] == "active"]
