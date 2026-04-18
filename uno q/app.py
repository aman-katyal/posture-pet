import asyncio, struct
from bleak import BleakClient, BleakScanner
from arduino.app_utils import Bridge

ESP32_ADDRESS = "90:70:69:35:2C:E2"
TARGET_SERVICE_UUID = "59462f12-9543-9999-12c8-58b459a2712d"
TARGET_CHAR_UUID = "33333333-2222-2222-1111-111100000000"

c = 0
def rc():
    global c

    c += 1
    if c > 180:
        c = 0
    return c

def notification_handler(sender, data):
    try:
        roll, pitch, yaw = struct.unpack('<fff', data)
        print(f"Roll: {roll:.2f} | Pitch: {pitch:.2f} | Yaw: {yaw:.2f}")
        Bridge.call("set_servo", 1, rc())
    except struct.error as e:
        print(f"Unpack error: {e} - Received {len(data)} bytes instead of 12.")

async def main(address):
    device = await BleakScanner.find_device_by_address(address, timeout=10)
    print(device)
    if not device:
        print("could not find device")
        return

    async with BleakClient(device) as client:
        print(f'Services found for device')
        print(f'\tDevice address:{device.address}')
        print(f'\tDevice name:{device.name}')

        found_sc = False
        print('\tServices:')
        for service in client.services:
            print()
            print(f'\t\tDescription: {service.description}')
            print(f'\t\tService: {service}')

            if not service.uuid == TARGET_SERVICE_UUID:
                continue

            print('\t\tCharacteristics:')
            for c in service.characteristics:
                print()
                print(f'\t\t\tUUID: {c.uuid}'),
                if not c.uuid == TARGET_CHAR_UUID:
                    continue

                found_sc = True
                print(f'\t\t\tDescription: {c.uuid}')
                print(f'\t\t\tHandle: {c.uuid}'),
                print(f'\t\t\tProperties: {c.uuid}')
                print('\t\tDescriptors:')
                for descrip in c.descriptors:
                    print(f'\t\t\t{descrip}')

        if found_sc:
            print(f"\n--- Found target! Subscribing to {TARGET_CHAR_UUID} ---")
            await client.start_notify(TARGET_CHAR_UUID, notification_handler)
            print("Listening for data... (Press Ctrl+C to stop)")
            try:
                while True:
                    await asyncio.sleep(1.0)
            except KeyboardInterrupt:
                print("\nStopping...")
            finally:
                await client.stop_notify(TARGET_CHAR_UUID)
        else:
            print(f"\nTarget characteristic {TARGET_CHAR_UUID} with 'notify' property was not found.")

    print("Disconnected")

asyncio.run(main(ESP32_ADDRESS))