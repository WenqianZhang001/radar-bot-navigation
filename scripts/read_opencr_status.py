#!/usr/bin/env python3
"""Read-only OpenCR identification/power/motor status; sends no register writes."""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import time


def main():
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',default='/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00')
    ap.add_argument('--output',type=Path)
    args=ap.parse_args();path=Path(args.port)
    if not path.exists() or subprocess.run(['fuser',str(path.resolve())],capture_output=True).returncode!=1:
        raise SystemExit('OpenCR absent or owned; do not open a competing serial reader')
    port=PortHandler(str(path));packet=PacketHandler(2.)
    if not port.openPort() or not port.setBaudRate(1000000):raise SystemExit('Cannot open OpenCR')
    result=dict(unix=time.time(),port=str(path.resolve()),read_only=True,registers={})
    try:
        model,comm,error=packet.ping(port,200)
        result['ping']=dict(model=model,communication=comm,error=error)
        for name,address,size in [('model_information',2,4),('firmware_register',6,1),('uptime_ms',10,4),
                ('debug_mode',14,1),('device_status',18,1),('heartbeat',19,1),('battery_voltage',42,4),
                ('motor_connected',148,1),('torque',149,1),('motor_velocity_position',128,16)]:
            data,comm,error=packet.readTxRx(port,200,address,size)
            result['registers'][name]=dict(address=address,bytes=data,communication=comm,error=error)
            if comm==COMM_SUCCESS and not error:
                value=struct.unpack('<4i',bytes(data)) if size==16 else int.from_bytes(bytes(data),'little')
                result[name]=value
        if 'battery_voltage' in result:result['battery_voltage_v']=result['battery_voltage']/100
        if 'device_status' in result:result['device_status_signed']=result['device_status']-256 if result['device_status']>127 else result['device_status']
    finally:port.closePort()
    if args.output:args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='registers'},indent=2))


if __name__=='__main__':main()
