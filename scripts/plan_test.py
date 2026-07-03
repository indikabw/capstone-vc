import math

def solve_ik_planar(r_target, z_target, alpha=1.57, ori_weight=0.001):
    x_rel = r_target - 0.012
    z_rel = z_target - 0.0595
    
    def fk(q2, q3, q4):
        x = math.cos(q2) * 0.024 + math.sin(q2) * 0.128 + math.cos(q2+q3) * 0.124 + math.cos(q2+q3+q4) * 0.126
        z = -math.sin(q2) * 0.024 + math.cos(q2) * 0.128 - math.sin(q2+q3) * 0.124 - math.sin(q2+q3+q4) * 0.126
        return x, z

    q = [0.968, -0.112, -0.055]
    lr = 0.5
    for i in range(1000):
        q2, q3, q4 = q
        x, z = fk(q2, q3, q4)
        
        ex = x - x_rel
        ez = z - z_rel
        e_ori = (q2 + q3 + q4) - alpha
        
        dq = 1e-5
        
        x_d2, z_d2 = fk(q2 + dq, q3, q4)
        g2 = ex * (x_d2 - x)/dq + ez * (z_d2 - z)/dq + ori_weight * e_ori
        
        x_d3, z_d3 = fk(q2, q3 + dq, q4)
        g3 = ex * (x_d3 - x)/dq + ez * (z_d3 - z)/dq + ori_weight * e_ori
        
        x_d4, z_d4 = fk(q2, q3, q4 + dq)
        g4 = ex * (x_d4 - x)/dq + ez * (z_d4 - z)/dq + ori_weight * e_ori
        
        q[0] -= lr * g2
        q[1] -= lr * g3
        q[2] -= lr * g4
        
        q[0] = max(-1.5, min(1.5, q[0]))
        q[1] = max(-1.5, min(1.4, q[1]))
        q[2] = max(-1.7, min(1.97, q[2]))
        
        if ex**2 + ez**2 < 1e-7:
            print(f"Converged in {i} steps.")
            break
            
    print(f"Final error: ex={ex:.5f}, ez={ez:.5f}")
    return q

# Test with r=0.28, z=-0.007
print("Alpha 1.57 (Forward):", solve_ik_planar(0.28, -0.007, alpha=1.57))
# Test with r=0.28, z=-0.007, pointing 45deg down (alpha=2.35)
print("Alpha 2.35 (Down-forward):", solve_ik_planar(0.28, -0.007, alpha=2.35))
