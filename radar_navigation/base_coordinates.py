"""Measured native TurtleBot axes -> radar-facing canonical body/world axes.

Supported-wheel isolation verified native right is canonical left, native left
is canonical right, and positive native wheel motion rolls both backward.
A half turn of both world and body preserves yaw and negates planar position.
"""
import math


def native_command(linear, angular):
    return -linear, angular


def canonical_wheels(left, right):
    return -right, -left


def canonical_pose(position, quaternion):
    x,y,z=position;qx,qy,qz,qw=quaternion
    # Rz(pi) * T_native * Rz(pi)^-1; equivalent quaternion conjugation.
    return (-x,-y,z),(-qx,-qy,qz,qw)


def canonical_vector(vector):
    x,y,z=vector
    return -x,-y,z


def canonical_covariance(covariance):
    if len(covariance)!=36:raise ValueError('Expected 6x6 covariance')
    signs=(-1,-1,1,-1,-1,1)
    return [float(covariance[i*6+j])*signs[i]*signs[j] for i in range(6) for j in range(6)]
