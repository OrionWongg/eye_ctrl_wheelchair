"""Conservative interpolation within measured five-ray velocity envelope."""
import numpy as np
# Ordered right -> forward -> left, at fitted rear-axis reference.
VELOCITIES=np.array([[0,-.800],[.384,-.590],[.548,0],[.394,.581],[0,.821]])
TARGETS=('right','right-forward','forward','left-forward','left')

def map_velocity(v,w,points):
    center=np.array(points['steering wheel center'],float)
    desired=np.array([max(0,float(v)),float(w)])
    for i in range(4):
        weights=np.linalg.solve(VELOCITIES[i:i+2].T,desired)
        if min(weights)>=-1e-8:
            weights=np.maximum(weights,0);weights/=max(1.,sum(weights))
            target=center+sum((weights[j]*(np.array(points[TARGETS[i+j]])-center) for j in range(2)))
            return tuple(int(value) for value in np.rint(target)), weights@VELOCITIES[i:i+2]
    raise ValueError('Velocity outside forward half-plane')


def map_input(x, y, points):
    """Direct control-plane position -> calibrated integer HID point + predicted twist.

    x points right, y points forward, both in normalized eight-step units.
    Clip along the ray to the polygon formed by measured calibration endpoints.
    """
    center=np.asarray(points['steering wheel center'],float)
    delta=np.array([np.clip(x,-1,1)*8,-np.clip(y,0,1)*8])
    rays=np.array([np.asarray(points[n])-center for n in TARGETS])
    for i in range(4):
        weights=np.linalg.solve(rays[i:i+2].T,delta)
        if min(weights)>=-1e-8:
            weights=np.maximum(weights,0);weights/=max(1.,sum(weights))
            point=np.rint(center+weights@rays[i:i+2]).astype(int)
            # Prediction describes the quantized command actually sent.
            actual_delta=point-center
            for j in range(4):
                actual_weights=np.linalg.solve(rays[j:j+2].T,actual_delta)
                if min(actual_weights)>=-1e-8:
                    return tuple(int(value) for value in point),actual_weights@VELOCITIES[j:j+2]
    raise ValueError('Invalid control-plane coordinate')
