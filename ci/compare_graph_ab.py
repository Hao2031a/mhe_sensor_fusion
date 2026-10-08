#!/usr/bin/env python3
"""Compare independently run ROS smoke profiles. Runtime is NOT hard-gated."""
import argparse, json
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('--baseline',type=Path,required=True)
p.add_argument('--optimized',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
b=json.loads(a.baseline.read_text()); o=json.loads(a.optimized.read_text())
assert b['pass'] and o['pass'], 'Both ROS smoke variants must pass checks'
fields=['graph_build_ms','ceres_only_ms','marginalization_ms','total_solve_ms','callback_wall_ms']
metrics={}
for field in fields:
    bp=b['realtime_profile'].get(field,{}).get('p99')
    op=o['realtime_profile'].get(field,{}).get('p99')
    metrics[field]={'baseline_p99_ms':bp,'optimized_p99_ms':op,
                    'ratio_optimized_over_baseline':op/bp if bp and op is not None else None}
report={'pass':True,'type':'real_ROS_node_synthetic_sensor_input_NOT_Gazebo_NOT_physical',
        'hard_performance_gate':False,'explanation':'Independent wall-time runs; compare distributions with matched rosbag for rigorous A/B.',
        'metrics':metrics,
        'baseline_graph_last':b.get('last_graph_status',[]),
        'optimized_graph_last':o.get('last_graph_status',[])}
a.output.parent.mkdir(parents=True,exist_ok=True)
a.output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
