import gps

# Create a GPS session
session = gps.gps(mode=gps.WATCH_ENABLE)

try:
    while True:
        # Get GPS data
        session.next()
        print(f"Latitude: {session.fix.latitude}")
        print(f"Longitude: {session.fix.longitude}")
        print(f"Altitude: {session.fix.altitude}")
except KeyboardInterrupt:
    pass
