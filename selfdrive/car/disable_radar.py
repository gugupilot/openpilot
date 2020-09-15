#!/usr/bin/env python3
import traceback

from tqdm import tqdm

import cereal.messaging as messaging
from panda import Panda
from panda.python.uds import UdsClient, NegativeResponseError, SESSION_TYPE, CONTROL_TYPE, MESSAGE_TYPE
from selfdrive.car.isotp_parallel_query import IsoTpParallelQuery
from selfdrive.swaglog import cloudlog

RADAR_ADDR = 0x7d0
EXT_DIAG_REQUEST = b'\x10\x03'
EXT_DIAG_RESPONSE = b'\x50\x03'
COM_CONT_REQUEST = b'\x28\x83\x03'
COM_CONT_RESPONSE = b''

def disable_radar(logcan, sendcan, bus, timeout=0.1, retry=5, debug=True):
  print(f"radar disable {hex(RADAR_ADDR)} ...")
  for i in range(retry):
    try:
      # enter extended diagnostic session
      query = IsoTpParallelQuery(sendcan, logcan, bus, [RADAR_ADDR], [EXT_DIAG_REQUEST], [EXT_DIAG_RESPONSE], debug=debug)
      for addr, dat in query.get_data(timeout).items():
        print("radar communication control disable tx/rx ...")
        # communication control disable tx and rx
        query = IsoTpParallelQuery(sendcan, logcan, bus, [RADAR_ADDR], [COM_CONT_REQUEST], [COM_CONT_RESPONSE], debug=debug)
        query = uds_client.communication_control(CONTROL_TYPE.DISABLE_RX_DISABLE_TX, MESSAGE_TYPE.NORMAL_AND_NETWORK_MANAGEMENT)
        query.get_data(0)
        # messages that work
        exit(0)
        return True
      print(f"radar disable retry ({i+1}) ...")
    except Exception:
      cloudlog.warning(f"radar disable exception: {traceback.format_exc()}")

  return False


if __name__ == "__main__":
  panda = Panda()
  panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  uds_client = UdsClient(panda, RADAR_ADDR, 0, timeout=0.1, debug=False)
  uds_client.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)

  import time
  sendcan = messaging.pub_sock('sendcan')
  logcan = messaging.sub_sock('can')
  time.sleep(1)
  disabled = disable_radar(logcan, sendcan, 0, debug=False)
  print(f"disabled: {disabled}")

  # messages that work
  #data = uds_client.communication_control(CONTROL_TYPE.DISABLE_RX_DISABLE_TX, MESSAGE_TYPE.NORMAL_AND_NETWORK_MANAGEMENT)
  #data = uds_client.communication_control(CONTROL_TYPE.DISABLE_RX_DISABLE_TX | 0x80, MESSAGE_TYPE.NORMAL_AND_NETWORK_MANAGEMENT)
  #exit(0)

  print("querying addresses ...")
  l = list(range(0x700))
  with tqdm(total=len(l)) as t:
    for i in l:
      ct = i >> 8
      mt = i & 0xFF
      t.set_description(f"{hex(ct)} - {hex(mt)}")
      try:
        data = uds_client.communication_control(ct, mt)
        print(f"\n{ct} - {mt}: success")
      except NegativeResponseError as e:
        if e.message != "COMMUNICATION_CONTROL - sub-function not supported" and e.message != "COMMUNICATION_CONTROL - request out of range":
          print(f"\n{ct} - {mt}: {e.message}")
      t.update(1)