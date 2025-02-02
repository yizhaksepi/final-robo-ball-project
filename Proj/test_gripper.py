import asyncio
from ur_robotiq_gripper import Gripper

async def log_info(gripper):
    print(f"Pos: {str(await gripper.get_current_position()): >3}  "
          f"Open: {await gripper.is_open(): <2}  "
          f"Closed: {await gripper.is_closed(): <2}  ")

async def run():
    gripper = Gripper('192.168.0.10')  # actual ip of the ur arm

    await gripper.connect()
    await gripper.activate()  # calibrates the gripper

    await gripper.move_and_wait_for_pos(255, 255, 255)
    await log_info(gripper)
    await gripper.move_and_wait_for_pos(0, 255, 255)
    await log_info(gripper)

asyncio.run(run())
