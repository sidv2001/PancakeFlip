import numpy as np 



def check_orientation(quaternion, translation, target_position=None, eps=1e-7):
    """
    Parameters:
        quaternion (np.array): 4 x 4 matrix
        translation (np.array): length 4 vector
        target_position:(np.array): 
        eps (float): for asserting closeness to target position
    Resturns:
        run_reorientation (bool): True if pancake is not in flipping position, False if 
    """
    raise NotImplementedError
