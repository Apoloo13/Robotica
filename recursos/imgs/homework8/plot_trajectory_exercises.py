import os
import numpy as np
import matplotlib.pyplot as plt

OUTDIR = os.path.join("docs", "recursos", "imgs", "homework_tp")
os.makedirs(OUTDIR, exist_ok=True)

# Exercise 1
t1 = np.linspace(0, 8, 801)

def q1_scalar(t):
    if t <= 0.5:
        return 0.5
    if t <= 1.5:
        x = (t - 0.5) / 1.0
        h00 = 2*x**3 - 3*x**2 + 1
        h10 = x**3 - 2*x**2 + x
        h01 = -2*x**3 + 3*x**2
        h11 = x**3 - x**2
        return h00*0.5 + h10*0 + h01*1.0 + h11*1.0
    if t <= 3.8:
        return 1.0 + (t - 1.5)
    if t <= 4.5:
        dt = 4.5 - 3.8
        x = (t - 3.8) / dt
        h00 = 2*x**3 - 3*x**2 + 1
        h10 = x**3 - 2*x**2 + x
        h01 = -2*x**3 + 3*x**2
        h11 = x**3 - x**2
        return h00*3.3 + h10*(1*dt) + h01*4.1 + h11*0
    if t <= 5.5:
        return 4.1
    if t <= 7.0:
        return 4.1 - 0.5*(t - 5.5)
    return 3.35

q1 = np.array([q1_scalar(t) for t in t1])
pts = {'A': 0.25, 'B': 1.0, 'C': 2.5, 'D': 4.0, 'E': 5.0, 'F': 6.25, 'G': 7.5}

plt.figure(figsize=(10, 5.4))
plt.plot(t1, q1, lw=2.5)
for label, t in pts.items():
    q = q1_scalar(t)
    plt.scatter([t], [q], s=350, facecolors='#f7c6c7', edgecolors='#d33', zorder=3)
    plt.text(t, q, label, ha='center', va='center', fontsize=16, weight='bold', color='#d33')
plt.title('Exercise 1 — Position graph used for the qualitative analysis', fontsize=15, weight='bold')
plt.xlabel('Time (s)')
plt.ylabel('Position (rad)')
plt.xlim(-0.2, 8.2)
plt.ylim(0.3, 4.3)
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(OUTDIR, 'exercise1_position_graph.png'), dpi=200)
plt.close()

# Exercise 2
t = np.array([0, 0.5, 2.5, 3.0])
v = np.array([0, 0.8, 0.8, 0])
plt.figure(figsize=(10, 5.2))
plt.plot(t, v, lw=2.5, marker='o')
plt.fill_between(t, v, 0, alpha=0.2)
plt.axvline(0.5, ls='--', lw=1, alpha=0.5)
plt.axvline(2.5, ls='--', lw=1, alpha=0.5)
plt.text(0.22, 0.1, r'$t_a=0.5\ \mathrm{s}$', fontsize=12)
plt.text(1.35, 0.1, r'$t_c=2.0\ \mathrm{s}$', fontsize=12)
plt.text(2.62, 0.1, r'$t_d=0.5\ \mathrm{s}$', fontsize=12)
plt.text(1.38, 0.28, r'$\Delta q=2.0\ \mathrm{rad}$', fontsize=18, alpha=0.75)
plt.title('Exercise 2 — Trapezoidal velocity profile and area under the curve', fontsize=15, weight='bold')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (rad/s)')
plt.xlim(-0.1, 3.1)
plt.ylim(-0.15, 1.0)
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(OUTDIR, 'exercise2_velocity_area.png'), dpi=200)
plt.close()

# Exercise 4
tt = np.linspace(0, 3, 601)
vv = np.piecewise(tt,
                  [tt <= 0.5, (tt > 0.5) & (tt <= 2.5), (tt > 2.5) & (tt <= 3)],
                  [lambda x: 1.6*x, 0.8, lambda x: 0.8 - 1.6*(x - 2.5)])
qq = np.piecewise(tt,
                  [tt <= 0.5, (tt > 0.5) & (tt <= 2.5), (tt > 2.5) & (tt <= 3)],
                  [lambda x: 0.8*x**2,
                   lambda x: 0.2 + 0.8*(x - 0.5),
                   lambda x: 1.8 + 0.8*(x - 2.5) - 0.8*(x - 2.5)**2])
plt.figure(figsize=(10, 5.2))
plt.plot(tt, qq, lw=2.5)
plt.axvline(0.5, ls='--', lw=1, alpha=0.5)
plt.axvline(2.5, ls='--', lw=1, alpha=0.5)
plt.text(0.08, 0.08, 'Quadratic segment', fontsize=11)
plt.text(1.0, 0.95, 'Linear segment', fontsize=11)
plt.text(2.53, 1.87, 'Quadratic segment', fontsize=11)
plt.title('Exercise 4 — Position obtained by integrating the trapezoidal velocity graph', fontsize=15, weight='bold')
plt.xlabel('Time (s)')
plt.ylabel('Position (rad)')
plt.xlim(-0.1, 3.1)
plt.ylim(-0.05, 2.1)
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(OUTDIR, 'exercise4_position_from_velocity.png'), dpi=200)
plt.close()

# Exercise 5
dq5, vmax5, a5 = 2.0, 1.0, 4.0
ta5 = vmax5 / a5
qmin5 = vmax5**2 / a5
tc5 = (dq5 - qmin5) / vmax5
T5 = 2*ta5 + tc5
tt = np.linspace(0, T5, 800)
mask1 = tt <= ta5
mask2 = (tt > ta5) & (tt <= ta5 + tc5)
mask3 = tt > ta5 + tc5

v5 = np.empty_like(tt)
v5[mask1] = a5*tt[mask1]
v5[mask2] = vmax5
v5[mask3] = vmax5 - a5*(tt[mask3] - (ta5 + tc5))

q5 = np.empty_like(tt)
q5[mask1] = 0.5*a5*tt[mask1]**2
q5[mask2] = 0.5*a5*ta5**2 + vmax5*(tt[mask2] - ta5)
q5[mask3] = (0.5*a5*ta5**2 + vmax5*tc5) + vmax5*(tt[mask3] - (ta5 + tc5)) - 0.5*a5*(tt[mask3] - (ta5 + tc5))**2

plt.figure(figsize=(10, 4.8))
plt.plot(tt, v5, lw=2.5)
for x in [ta5, ta5 + tc5, T5]:
    plt.axvline(x, ls='--', lw=1, alpha=0.5)
plt.title('Exercise 5 — Velocity profile', fontsize=15, weight='bold')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (rad/s)')
plt.xlim(0, T5)
plt.ylim(0, 1.15)
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(OUTDIR, 'exercise5_velocity_profile.png'), dpi=200)
plt.close()

plt.figure(figsize=(10, 4.8))
plt.plot(tt, q5, lw=2.5)
for x in [ta5, ta5 + tc5, T5]:
    plt.axvline(x, ls='--', lw=1, alpha=0.5)
plt.title('Exercise 5 — Position profile', fontsize=15, weight='bold')
plt.xlabel('Time (s)')
plt.ylabel('Position (rad)')
plt.xlim(0, T5)
plt.ylim(0, 2.05)
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(OUTDIR, 'exercise5_position_profile.png'), dpi=200)
plt.close()

# Exercise 6
dq6, T6 = 3.0, 3.0
ta_tri = T6/2
vp = 2*dq6/T6
a_tri = vp/ta_tri

ta_trap = 0.5
tc6 = T6 - 2*ta_trap
vmax6 = dq6 / (tc6 + ta_trap)
a_trap = vmax6 / ta_trap

tt = np.linspace(0, T6, 900)
vtri = np.piecewise(tt, [tt <= ta_tri, tt > ta_tri],
                    [lambda x: a_tri*x, lambda x: vp - a_tri*(x - ta_tri)])
vtrap = np.piecewise(tt,
                     [tt <= ta_trap, (tt > ta_trap) & (tt <= ta_trap + tc6), tt > ta_trap + tc6],
                     [lambda x: a_trap*x, vmax6, lambda x: vmax6 - a_trap*(x - (ta_trap + tc6))])

atri = np.piecewise(tt, [tt < ta_tri, tt == ta_tri, tt > ta_tri], [a_tri, np.nan, -a_tri])
atrap = np.piecewise(tt,
                     [tt < ta_trap, (tt >= ta_trap) & (tt <= ta_trap + tc6), tt > ta_trap + tc6],
                     [a_trap, 0, -a_trap])

plt.figure(figsize=(10, 5))
plt.plot(tt, vtri, lw=2.5, label='Triangular')
plt.plot(tt, vtrap, lw=2.5, label='Trapezoidal')
plt.title('Exercise 6 — Velocity profiles comparison', fontsize=15, weight='bold')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (rad/s)')
plt.xlim(0, T6)
plt.ylim(0, 2.15)
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(OUTDIR, 'exercise6_velocity_comparison.png'), dpi=200)
plt.close()

plt.figure(figsize=(10, 5))
plt.plot(tt, atri, lw=2.5, label='Triangular')
plt.plot(tt, atrap, lw=2.5, label='Trapezoidal')
plt.title('Exercise 6 — Acceleration profiles comparison', fontsize=15, weight='bold')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (rad/s²)')
plt.xlim(0, T6)
plt.ylim(-2.7, 2.7)
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(OUTDIR, 'exercise6_acceleration_comparison.png'), dpi=200)
plt.close()
