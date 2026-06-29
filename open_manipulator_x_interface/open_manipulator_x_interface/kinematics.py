"""
kinematics.py — Cinemática del OpenMANIPULATOR-X (4 GDL).

- fkin(q): cinemática directa analítica. Es un port EXACTO de la fkin() de
  `hw_fkin_node.cpp` / `open_manx_fkin.m` del paquete torque_control, de modo
  que la pose mostrada coincide con la de los laboratorios.
- jacobian(q): Jacobiano 4x4 por diferencias finitas de fkin.
- ik_step(q, dpose): paso de cinemática inversa por mínimos cuadrados amortiguados
  (DLS). Ideal para el jog cartesiano incremental: se le pide un desplazamiento
  (dx, dy, dz) del efector y devuelve una nueva configuración articular,
  manteniendo la orientación phi y respetando los límites.

Vector articular: q = [q1, q2, q3, q4]  (q1 = giro de la base).
Pose del efector: p = [x, y, z, phi]  con phi = q2 + q3 + q4.
"""

import math

import numpy as np

from . import config

# Geometría (m), idéntica a hw_fkin_node.cpp:104
X_BASE = 0.012
Z_BASE = 0.017 + 0.0595
X23 = 0.024
Z23 = 0.128
L34 = 0.124
L4E = 0.126


def fkin(q):
    """Cinemática directa. q = [q1,q2,q3,q4] -> np.array([x, y, z, phi])."""
    q1, q2, q3, q4 = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    r = (X23 * math.cos(q2) + Z23 * math.sin(q2)
         + L34 * math.cos(q2 + q3)
         + L4E * math.cos(q2 + q3 + q4))
    z = (Z_BASE
         + (-X23 * math.sin(q2) + Z23 * math.cos(q2))
         - L34 * math.sin(q2 + q3)
         - L4E * math.sin(q2 + q3 + q4))
    x = X_BASE + r * math.cos(q1)
    y = r * math.sin(q1)
    phi = q2 + q3 + q4
    return np.array([x, y, z, phi])


def jacobian(q, eps=1e-6):
    """Jacobiano 4x4 d(pose)/d(q) por diferencias finitas centradas."""
    q = np.asarray(q, dtype=float)
    J = np.zeros((4, 4))
    for i in range(4):
        dq = np.zeros(4)
        dq[i] = eps
        J[:, i] = (fkin(q + dq) - fkin(q - dq)) / (2.0 * eps)
    return J


def clamp_to_limits(q):
    """Recorta q a los límites articulares activos (config.joint_limits())."""
    out = np.array(q, dtype=float)
    for i, name in enumerate(config.JOINT_NAMES):
        out[i] = config.clamp_joint(name, out[i])
    return out


def within_limits(q, margin=0.0):
    for i, name in enumerate(config.JOINT_NAMES):
        lo, hi = config.joint_limits()[name]
        if q[i] < lo - margin or q[i] > hi + margin:
            return False
    return True


def ik_step(q_seed, dpose, hold_phi=True, damping=0.02, max_dq=0.25):
    """
    Un paso de IK incremental (DLS) para jog cartesiano.

    q_seed : configuración articular actual [rad] (semilla).
    dpose  : desplazamiento deseado del efector [dx, dy, dz] [m].
    hold_phi: si True, mantiene la orientación phi (target dphi = 0).

    Devuelve la nueva q (recortada a límites) o None si el resultado quedaría
    fuera de límites o el cálculo es singular.
    """
    q_seed = np.asarray(q_seed, dtype=float)
    target = np.array([dpose[0], dpose[1], dpose[2], 0.0])  # [dx,dy,dz,dphi]

    J = jacobian(q_seed)
    if not hold_phi:
        # Solo posición: usa las 3 primeras filas (deja phi libre).
        Jp = J[0:3, :]
        e = target[0:3]
        JJt = Jp @ Jp.T + (damping ** 2) * np.eye(3)
        try:
            dq = Jp.T @ np.linalg.solve(JJt, e)
        except np.linalg.LinAlgError:
            return None
    else:
        JJt = J @ J.T + (damping ** 2) * np.eye(4)
        try:
            dq = J.T @ np.linalg.solve(JJt, target)
        except np.linalg.LinAlgError:
            return None

    # Limita el tamaño del paso articular (suavidad / seguridad).
    n = np.linalg.norm(dq)
    if n > max_dq:
        dq = dq * (max_dq / n)

    q_new = q_seed + dq
    if not within_limits(q_new, margin=1e-3):
        # No comandamos si nos saldríamos de límites: el operador lo notará.
        return None
    return clamp_to_limits(q_new)


def ik_pose(target_pose, q_seed, iters=80, tol=1e-4):
    """
    IK iterativa (DLS) hacia una pose objetivo [x,y,z,phi]. Útil para tests o
    para llevar el efector a una pose concreta. Devuelve q o None si no converge.
    """
    q = np.asarray(q_seed, dtype=float).copy()
    target = np.asarray(target_pose, dtype=float)
    for _ in range(iters):
        e = target - fkin(q)
        if np.linalg.norm(e) < tol:
            return clamp_to_limits(q)
        J = jacobian(q)
        JJt = J @ J.T + (0.02 ** 2) * np.eye(4)
        try:
            dq = J.T @ np.linalg.solve(JJt, e)
        except np.linalg.LinAlgError:
            return None
        # paso amortiguado
        q = q + np.clip(dq, -0.2, 0.2)
    return clamp_to_limits(q) if np.linalg.norm(target - fkin(q)) < 5e-3 else None


def _selftest():
    print('fkin(0) =', np.round(fkin([0, 0, 0, 0]), 4),
          '(esperado ~ [0.286, 0.0, 0.2045, 0.0])')
    # Round-trip: parte de una q, calcula pose, recupera q por IK.
    q_true = np.array([0.3, 0.4, -0.5, 0.2])
    p = fkin(q_true)
    q_rec = ik_pose(p, q_seed=np.zeros(4))
    if q_rec is None:
        print('IK no convergió')
    else:
        print('pose objetivo   :', np.round(p, 4))
        print('pose recuperada :', np.round(fkin(q_rec), 4))
        print('error de pose   :', np.round(np.linalg.norm(p - fkin(q_rec)), 6))
    # Jog: un pequeño paso en +x debería aumentar x manteniendo phi.
    q0 = np.array([0.0, 0.3, -0.4, 0.1])
    q1 = ik_step(q0, [0.01, 0.0, 0.0])
    if q1 is not None:
        print('jog +1cm en x: dpose =', np.round(fkin(q1) - fkin(q0), 4))


if __name__ == '__main__':
    _selftest()
