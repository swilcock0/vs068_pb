import numpy as np
import pybullet as p
import vs068_pb.config as config

'''pyBullet convenience functions'''

def Disconnect():
    ''' Disconnect any and all instances of bullet '''
    for i in range(100):
        try:
            p.disconnect(i)
        except:
            break # Don't whinge about non-existent servers

def Camera(cid, botId, linkID, distance=0.2):
    ''' Rig a camera to a link '''
    # Turn off using params at top
    if not config.CAMERA_ACTIVATED:
        return
    
    # Get joint pose
    pos, orn = p.getLinkState(botId, linkID)[0:2]

    # Extract vectors from the rotation transformation
    rot_transform = p.getMatrixFromQuaternion(orn)
    forward_vector = [rot_transform[0], rot_transform[3], rot_transform[6]]
    up_vector = [rot_transform[2], rot_transform[5], rot_transform[8]]

    # Get a camera direction
    camera_target = [
            pos[0] + forward_vector[0],
            pos[1] + forward_vector[1],
            pos[2] + forward_vector[2]]

    # Form viewmatrix and projection matrix
    viewMatrix = p.computeViewMatrix(
                        pos,
                        camera_target,
                        up_vector)

    projectionMatrix = p.computeProjectionMatrixFOV(
                        fov=45.0,
                        aspect=1.0,
                        nearVal=0.1,
                        farVal=3.1)

    # Call the camera
    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                        width=224, 
                        height=224,
                        viewMatrix=viewMatrix,
                        projectionMatrix=projectionMatrix)


''' Helper utils - nicked from caelan/pybullet_planning '''

def invert(pose):
    (point, quat) = pose
    return p.invertTransform(point, quat)

def multiply(*poses):
    pose = poses[0]
    for next_pose in poses[1:]:
        pose = p.multiplyTransforms(pose[0], pose[1], *next_pose)
    return pose


def get_length(vec, norm=2):
    return np.linalg.norm(vec, ord=norm)

def get_difference(p1, p2):
    assert len(p1) == len(p2)
    return np.array(p2) - np.array(p1)

def get_distance(p1, p2, **kwargs):
    return get_length(get_difference(p1, p2), **kwargs)

def get_joint_limits(cid=0, botId=0, joint=0):
    # TODO: make a version for several joints?
    joint_info = p.getJointInfo(botId, joint, physicsClientId=cid)
    return joint_info[8], joint_info[9]

def get_min_limit(botId, joint):
    # TODO: rename to min_position
    return get_joint_limits(botId, joint)[0]

def get_min_limits(botId, joints):
    return [get_min_limit(botId, joint) for joint in joints]

def get_max_limit(botId, joint):
    return get_joint_limits(botId, joint)[1]

def get_max_limits(botId, joints):
    return [get_max_limit(botId, joint) for joint in joints]  

def get_joint_positions(botId, joint_indices, cid=0):
    positions = []
    for i in range(len(joint_indices)):
        positions.append(p.getJointStates(botId, joint_indices, cid)[0])
    return positions