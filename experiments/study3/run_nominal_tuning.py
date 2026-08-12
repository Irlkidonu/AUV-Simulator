#!/usr/bin/env python3
"""Development-only nominal guard for Study 3 adaptive bundle selection."""
from __future__ import annotations
import importlib.util,json,statistics,sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("study3_development",HERE/"run_development.py")
dev=importlib.util.module_from_spec(spec);sys.modules[spec.name]=dev;spec.loader.exec_module(dev)

root=31_300_400
winner=json.loads((dev.OUTPUT/"fixed_v2_summary.json").read_text())["winner"]
bundles={
 "balanced":{**winner,"prediction_horizon_s":10.,"optical_quality_floor":.25,"usable_probability_boundary":.35,"trend_confirmation_frames":3,"minimum_cumulative_quality_decline":.18},
 "conservative":{**winner,"prediction_horizon_s":8.,"optical_quality_floor":.22,"usable_probability_boundary":.30,"trend_confirmation_frames":3,"minimum_cumulative_quality_decline":.22},
 "early":{**winner,"prediction_horizon_s":12.,"optical_quality_floor":.28,"usable_probability_boundary":.40,"trend_confirmation_frames":3,"minimum_cumulative_quality_decline":.15},
 "confirmed":{**winner,"prediction_horizon_s":10.,"optical_quality_floor":.25,"usable_probability_boundary":.35,"trend_confirmation_frames":4,"minimum_cumulative_quality_decline":.18}}
tasks=[("adaptive_nominal",root,"S3_NOMINAL",i,"predictive",bid,c)
       for bid,c in bundles.items() for i in range(15)]
tasks += [("adaptive_nominal",root,"S3_NOMINAL",i,"reactive","reference",winner) for i in range(15)]
results,elapsed=dev.run_tasks(tasks,4)
reactive=[r for r in results if r["policy"]=="reactive"]
reference=statistics.mean(r["unnecessary_interventions"] for r in reactive)
scores={}
for bid in bundles:
 rows=[r for r in results if r["configuration_id"]==bid]
 scores[bid]={"mean_unnecessary_interventions":statistics.mean(r["unnecessary_interventions"] for r in rows),
              "difference_from_reactive":statistics.mean(r["unnecessary_interventions"] for r in rows)-reference,
              "completion_rate":statistics.mean(r["completed"] for r in rows)}
eligible=[x for x in bundles if scores[x]["difference_from_reactive"]<=1.0]
summary=dev.save_summary("adaptive_nominal",root,results,elapsed,
    {"reactive_mean_unnecessary_interventions":reference,"bundle_scores":scores,
     "eligible_bundles":eligible})
print(json.dumps(summary,indent=2,sort_keys=True))
