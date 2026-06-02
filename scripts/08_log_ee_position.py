"""Log left and right end-effector site positions."""

from stage2_common import OUTPUT_DIR, describe_sites, make_ready_model

model, data, _home_ctrl = make_ready_model()
summary = describe_sites(model, data)

print("===== END-EFFECTOR POSITIONS =====")
print(summary)

(OUTPUT_DIR / "ee_position.txt").write_text(summary, encoding="utf-8")
print("\nSaved to outputs/ee_position.txt")
