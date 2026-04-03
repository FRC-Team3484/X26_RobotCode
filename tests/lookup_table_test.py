from src.constants import FeedTargetSubsystemConstants

def test_lookup_table():
    hub_distances = FeedTargetSubsystemConstants.HUB_DISTANCES
    hub_speeds = FeedTargetSubsystemConstants.HUB_RPM
    hub_times = FeedTargetSubsystemConstants.HUB_FLIGHT_TIME

    feed_distances = FeedTargetSubsystemConstants.FEED_DISTANCES
    feed_speeds = FeedTargetSubsystemConstants.FEED_RPM
    feed_times = FeedTargetSubsystemConstants.FEED_FLIGHT_TIME

    assert len(hub_distances) == len(hub_speeds) == len(hub_times), f"Hub lookup table lengths do not match: distances:{len(hub_distances)}, speeds:{len(hub_speeds)}, times:{len(hub_times)}"
    assert len(feed_distances) == len(feed_speeds) == len(feed_times), f"Feed lookup table lengths do not match: distances:{len(feed_distances)}, speeds:{len(feed_speeds)}, times:{len(feed_times)}"