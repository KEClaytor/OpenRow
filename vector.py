import math

#################
## Vector Math ##
#################
def vector_mean(vector_list):
    """ Average a list of 3-vectors

    [(x1, y1, z1), ..., (xn, yn, zn)] -> (xm, ym, zm)
    """
    vlen = len(vector_list)
    mv = tuple(
        sum(v[ii] for v in vector_list) / vlen for ii in range(3)
    )
    return mv

def vector_multiply(s, v):
    """ Multiply a vector by a scalar
    """
    return tuple(s * vi for vi in v)

def vector_subtract(va, vb):
    """ Vector version of a - b
    """
    return tuple(a - b for a, b in zip(va, vb))

def dot(xv, yv):
    """ Dot product: xv.yv
    """
    return sum(x*y for x, y in zip(xv, yv))

def norm(v):
    """ Euclidean norm: sqrt(v.v)
    """
    mv = math.sqrt(dot(v, v))
    if mv == 0.0:
        return (0, 0, 0)
    else:
        return tuple(vi/mv for vi in v)
