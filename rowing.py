
import math
import vector

##############
## Geo Math ##
##############

def calculate_distance(lat0, lon0, lat1, lon1):
    """ Compute distance in m between to latitude and longitude points

    Approximation only, good if
        1. the points are not separated too much (we're talking km usually)
        2. you are not near the poles
    """
    deglen = 111.1*1e3
    x = lat1 - lat0
    y = (lon1 - lon0)*math.cos(lat0*math.pi/180)
    d = deglen*math.sqrt(x*x + y*y)
    return d


##############
## Closures ##
##############


def GravityRemover(history=5):
    """ Track and remove the average component of N acceleration vectors

    Internally track the last N accleration vectors.
    Compute the average acceleration vector from these.
    Then subtract that from the current acceleration vector.
    And return this reduced acceleration vector.
    """
    hv = []
    def _call(v):

        if len(hv) < 1:
            v_reduced = (0, 0, 0)
        else:
            # Compute the direction of average acceleration
            a_hat = vector.norm(vector.vector_mean(hv))
            # Remove the component parallel to average accleration
            v_parallel = vector.vector_multiply(vector.dot(v, a_hat), a_hat)
            v_reduced = vector.vector_subtract(v, v_parallel)
        # Update the histopy elements with the new vector
        hv.append(v)
        if len(hv) > history:
            hv.pop(0)
        return v_reduced

    return _call


def HistoryThresholder(threshold=20.0, mode="rising"):
    """ Pulse when something exceeds our threshold and we're on a rising edge
    """
    if mode == "rising":
        comparitor = lambda x0, x1, thresh: x1 > thresh and x0 < thresh
    elif mode == "falling":
        comparitor = lambda x0, x1, thresh: x1 < thresh and x0 > thresh
    else:
        raise TypeError("mode can be 'rising' or 'falling'")

    previous = [0, 0]
    def _call(value):
        previous.append(value)
        previous.pop(0)

        return comparitor(previous[0], previous[1], threshold)

    return _call


def StrokeRate(history_num=3, history_time=20):
    """ Compute stroke rate

    We want a fairly updated (instaneous) stroke rate
    But we also don't want to compute rates from strokes separated by a long time.
    Inputs:
        history_num = Drop more than history_num strokes
        history_time = Drop strokes older than this time
    Internally:
        stroke_data = List of tuples [(seconds, distance)]
    """
    # A single value becomes a local variable
    stroke_data = []

    def _call(stroke, seconds, distance):
        """ Update the stroke data and return current rate and split.
            stroke = has a stroke occured?
            seconds = seconds since we started
            distance = current distance travelled
        """

        # Only update the data on a stroke event
        if stroke:
            stroke_data.append((seconds, distance))

        # Drop points based on number
        while len(stroke_data) > history_num:
            stroke_data.pop(0)

        # Drop points based on time
        while stroke_data and (seconds - stroke_data[0][0] > history_time):
            stroke_data.pop(0)

        # Update our rate
        strokes = len(stroke_data)
        if strokes > 1:
            time0, dist0 = stroke_data[0]
            time1, dist1 = stroke_data[-1]
            delta_t = time1 - time0
            delta_d = dist1 - dist0

            # Stroke rate = strokes / delta_t [s] * 60 [s]/minute
            stroke_rate = round(60 * strokes / delta_t, 1)

            # Split = time for 500 m
            if delta_d > 0:
                split_time = 500.0 * delta_t / delta_d
            else:
                split_time = 0
        else:
            stroke_rate, split_time = 0, 0

        return (stroke_rate, split_time)

    return _call


def DistanceTracker():
    """ Compute distance between current and last point
    """
    ilat = [None, None]
    ilon = [None, None]

    def _call(lat, lon):
        # Update the lat / lon arrays
        ilat.append(lat)
        ilat.pop(0)
        ilon.append(lon)
        ilon.pop(0)

        if (ilat[0] is not None) and (ilat[1] is not None):
            distance_travelled = calculate_distance(ilat[0], ilon[0], ilat[1], ilon[1])
        else:
            distance_travelled = 0

        return distance_travelled

    return _call
