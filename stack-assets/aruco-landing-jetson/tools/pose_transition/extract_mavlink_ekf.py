#!/usr/bin/env python3
"""Read recorded PX4 MAVLink estimator telemetry without connecting to hardware."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import struct
import rosbag
from pymavlink.dialects.v20 import common as mav


def clean(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,list):return [clean(v) for v in value]
    return value


def extract(bag_path,output):
    counts=Counter();status=[];odom=[];texts=[]
    with rosbag.Bag(str(bag_path)) as bag:
        for _,msg,t in bag.read_messages(topics=['/mavlink/from']):
            counts[msg.msgid]+=1
            if msg.msgid not in (230,253,331):continue
            cls=mav.mavlink_map[msg.msgid]
            payload=struct.pack('<'+'Q'*len(msg.payload64),*msg.payload64)[:msg.len]
            values=cls.unpacker.unpack(payload[:cls.unpacker.size].ljust(cls.unpacker.size,b'\x00'))
            if msg.msgid==331:
                odom.append(dict(ros_time=t.to_sec(),time_usec=values[0],xyz=list(values[1:4]),
                    frame_id=values[-5],child_frame_id=values[-4],reset_counter=values[-3],
                    estimator_type=values[-2],quality=values[-1],payload_length=msg.len))
            else:
                row=dict(zip(cls.ordered_fieldnames,values));row['ros_time']=t.to_sec()
                if msg.msgid==253:
                    row['text']=row['text'].split(b'\x00')[0].decode(errors='replace');texts.append(row)
                else:status.append(row)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(clean(dict(message_counts=dict(counts),estimator_status=status,
        odometry=odom,statustext=texts)),indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('bag',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();extract(args.bag,args.output)
