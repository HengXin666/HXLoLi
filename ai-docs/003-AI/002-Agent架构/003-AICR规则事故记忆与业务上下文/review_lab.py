"""运行千条规则规划、证据包与最小事故回归；不调用云端模型。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from review_core import RuleStore, change, expert_rules, knowledge, library, plan, replacement_survives, run


def checks_planning() -> list[str]:
    rules = library()
    assert len(rules) == len({item["id"] for item in rules}) == 1200
    envelope = change("grant")
    applicable = plan(rules, envelope)
    assert len(applicable) == 21 and not envelope["model_issues"]
    ranked = sorted(applicable, key=lambda rule: rule["id"] == "GRANT")
    assert "GRANT" not in {item["id"] for item in ranked[:20]}
    report = run(rules, envelope, knowledge(), 100)
    assert "GRANT" in {item["rule_id"] for item in report["results"]}
    assert report["gate"] == "needs_review" and not report["complete"]
    empty = run(rules, envelope | {"paths": ["renamed/new_entry.py"]}, knowledge(), 100)
    assert empty["applicable_count"] == 0 and empty["gate"] == "needs_review"
    return ["1200_unique_rules", "empty_model_issue_still_plans", "top20_counterexample",
            "critical_rule_preserved", "await_model_is_not_pass", "unmapped_scope_is_not_pass"]


def checks_regression() -> list[str]:
    assert not replacement_survives("user_workspace")
    assert replacement_survives("identity")
    rules = [expert_rules()[0]]
    old = run(rules, change("grant"), knowledge(), 2, "user_workspace")
    fixed = run(rules, change("grant"), knowledge(), 2, "identity")
    short = run(rules, change("grant"), knowledge(), 1)
    assert old["gate"] == "block" and fixed["gate"] == "pass"
    assert short["results"][0]["status"] == "skipped_budget" and not short["complete"]
    return ["old_grant_bug_fails", "identity_fix_preserves_regrant", "budget_shortfall_visible"]


def checks_knowledge() -> list[str]:
    rules, envelope = library(), change("retention")
    current = knowledge()
    normal = run(rules, envelope, current, 2)
    assert normal["packets"][0]["question"].startswith("删除 creator")
    current["retention_contract"]["revision"] = "business-r4"
    current["retention_contract"]["text"] = "新规则"
    original = next(e for e in normal["packets"][0]["evidence"] if e["id"] == "retention_contract")
    assert original["revision"] == "business-r2" and original["text"] != "新规则"
    for mutation in ["restricted", "expired", "revoked"]:
        sources = knowledge()
        item = sources["retention_contract"]
        if mutation == "restricted": item["acl"] = ["owner"]
        elif mutation == "expired": item["valid_until"] = 2
        else: item["state"] = "retired"
        report = run(rules, envelope, sources, 2)
        packet = report["packets"][0]
        assert "retention_contract" in packet["missing"]
        assert not any(e["id"] == "retention_contract" for e in packet["evidence"])
        assert report["results"][0]["status"] == "unknown" and report["gate"] == "needs_review"
    assert normal["packets"][0]["missing"] == []
    return ["expert_question_and_evidence", "acl_filtered_before_packet", "expired_is_unknown",
            "revoked_is_unknown", "old_packet_preserves_revision"]


def checks_verdicts() -> list[str]:
    rules, envelope = library(), change("retention")
    forged = {"RETENTION": {"status": "pass", "evidence_ids": ["nonexistent"]}}
    invalid = run(rules, envelope, knowledge(), 2, verdicts=forged)
    assert invalid["results"][0]["status"] == "error"
    valid = {"RETENTION": {"status": "pass", "evidence_ids": ["code", "retention_contract"]}}
    unverified = run(rules, envelope, knowledge(), 2, verdicts=valid)
    assert unverified["complete"] and unverified["gate"] == "needs_review"
    assert unverified["results"][0]["validation_pending"]
    store = RuleStore()
    first = store.reconcile("repo:file:section", "severity=high")
    store.reconcile("repo:file:section", "severity=critical")
    store.reconcile("repo:file:section", None)
    assert first["revision"] == 1 and len(store.history[first["id"]]) == 3
    assert not store.active() and store.history[first["id"]][0]["content"] == "severity=high"
    return ["invalid_citation_rejected", "model_pass_requires_validation", "source_update_delete_history"]


def self_test() -> dict:
    names = checks_planning() + checks_regression() + checks_knowledge() + checks_verdicts()
    return {"ok": True, "checks": len(names), "passed": names,
            "limits": "合成规则和最小逻辑回归；没有生产数据库、AST 或模型理解质量实测"}


def parser() -> argparse.ArgumentParser:
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--self-test", action="store_true")
    cli.add_argument("--case", choices=["grant", "retention", "repair"], default="retention")
    cli.add_argument("--budget", type=int, default=22)
    cli.add_argument("--grant-mode", choices=["identity", "user_workspace"], default="identity")
    cli.add_argument("--verdicts", type=Path, help="从外部模型/人工端口导入 JSON；仍需独立验证")
    cli.add_argument("--rules-out", type=Path)
    cli.add_argument("--out", type=Path)
    cli.add_argument("--enforce-gate", action="store_true")
    return cli


def main() -> int:
    args = parser().parse_args()
    if args.budget < 0:
        raise SystemExit("budget 必须非负")
    rules = library()
    if args.rules_out:
        args.rules_out.write_text(json.dumps(rules, ensure_ascii=False, indent=2))
    verdicts = json.loads(args.verdicts.read_text()) if args.verdicts else {}
    if not isinstance(verdicts, dict):
        raise SystemExit("verdicts 必须是以 rule ID 为键的对象")
    report = self_test() if args.self_test else run(
        rules, change(args.case), knowledge(), args.budget, args.grant_mode, verdicts)
    output = json.dumps(report, ensure_ascii=False, indent=2)
    if args.out: args.out.write_text(output)
    else: print(output)
    if args.enforce_gate and report.get("gate") != "pass":
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
