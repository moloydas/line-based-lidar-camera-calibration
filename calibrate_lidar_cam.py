import torch
import kornia
import numpy as np
import csv
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
from torch import autograd
from scipy.spatial.transform import Rotation

fx = 692.653256; fy = 692.653256; cx = 629.321381; cy = 330.685425
P = np.array([  [ fx, 0.0, cx, 0.000000],
                [0.0,  fy, cy, 0.000000],
                [0.0, 0.0,1.0, 0.000000] ])
total_rot = 0
total_trans = 0

def convert_pix_coord_2_3d(pix):
    px = (pix[0] - cx)/fx
    py = (pix[1] - cy)/fy
    return np.array([px,py,1])

def project_3d_pt_on_img(P, w_R_c, w_t_c, pt_3d):
    pt_3d = np.append(pt_3d, [1]).reshape(4,1)
    w_t_c = np.append(w_t_c, [1])
    w_T_c = np.append(w_R_c, np.array([0,0,0]).reshape(1,3),axis=0)
    w_T_c = np.append(w_T_c, w_t_c.reshape(4,1),axis=1)
    c_T_w = np.linalg.inv(w_T_c)
    tran_pt =  np.dot(c_T_w, pt_3d)
    proj_3d_pt = np.dot(P, tran_pt).reshape(3,1)
    pix = np.array([proj_3d_pt[0]/proj_3d_pt[2],proj_3d_pt[1]/proj_3d_pt[2]]).reshape(2,1)
    return pix

def cost_function_rot(N, w_q_c, D):
    w_R_c = kornia.quaternion_to_rotation_matrix(w_q_c)
    D_rotated = torch.matmul(w_R_c.T, D.double())
    B = N * D_rotated
    B = torch.sum(B,dim=0)
    B = B**2
    return torch.sum(B)

def cost_function_trans(N, w_q_c, w_T_c, pt_m):
    c_R_w = kornia.quaternion_to_rotation_matrix(w_q_c).T
    m_rotated = torch.matmul(c_R_w.double(), pt_m.double())
    rt = torch.matmul(c_R_w.double(), w_T_c.double())
    m_trans = torch.add(m_rotated, -rt)
    B = N*m_trans
    B = torch.sum(B, axis=0)
    B = B**2
    return torch.sum(B)

def get_img_lines_normal(img_lines_file):
    normals = []
    lines = []
    with open(img_lines_file, 'r') as line_file:
        reader = csv.reader(line_file, delimiter=',')
        for row in reader:
            pt_a = np.array([float(a) for a in row[:2]])
            pt_b = np.array([float(a) for a in row[2:4]])
            N_ab = np.cross(convert_pix_coord_2_3d(pt_a), convert_pix_coord_2_3d(pt_b))
            N_ab = N_ab/(np.sqrt(np.dot(N_ab,N_ab)))

            normals.append(N_ab)
            lines.append([pt_a, pt_b])

    del lines[0]
    del normals[0]
    return lines, np.array(normals).T

def get_3d_lines_dir(pc_lines_file):
    lines_3d = []
    dirs = []
    with open(pc_lines_file, 'r') as line_file:
        reader = csv.reader(line_file, delimiter=',')
        for row in reader:
            pt_a = np.array([float(a) for a in row[:3]])
            pt_b = np.array([float(a) for a in row[3:6]])
            dir_ab = (pt_a - pt_b)
            dir_ab /= np.sqrt(np.dot(dir_ab, dir_ab))
            dirs.append(dir_ab)
            lines_3d.append([pt_a, pt_b])

    del lines_3d[0]
    del dirs[0]
    return lines_3d, np.array(dirs).T

itr = 0
def vis_pts(final_img, P, lines, w_R_c, w_t_c, show=True, itr_flag=False, wait_time=30):
    global itr
    global total_trans
    global total_rot

    if itr_flag:
        end_pnt_color = [0,0,255]
        line_color = (255,0,0)

        cv2.putText(final_img,"itr: "+str(itr*100), (50, 600), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0,0,255), 2, cv2.LINE_AA)
        cv2.putText(final_img,"trans_error: "+str(round(total_trans.item(), 6)), (50, 630), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0,0,255), 2, cv2.LINE_AA)
        cv2.putText(final_img,"rot_error: "+str(round(total_rot.item(), 6)), (50, 660), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0,0,255), 2, cv2.LINE_AA)
        itr += 1
    else:
        end_pnt_color = [255,0,0]
        line_color = (0,0,255)

    for line in lines:
        pnt_a = project_3d_pt_on_img(P, w_R_c, w_t_c, line[0].flatten())
        pnt_b = project_3d_pt_on_img(P, w_R_c, w_t_c, line[1].flatten())
        cv2.circle(final_img, (int(pnt_a[0,0]), int(pnt_a[1,0])), 6, end_pnt_color, -1)
        cv2.circle(final_img, (int(pnt_b[0,0]), int(pnt_b[1,0])), 6, end_pnt_color, -1)

        final_img = cv2.line(final_img, tuple(pnt_a), tuple(pnt_b), line_color, 2)

    if show:
        cv2.imshow('img', cv2.resize(final_img, (900,600)))
        cv2.imwrite('results/'+str(itr*100)+'.png', final_img)
        cv2.waitKey(wait_time)

if __name__ == '__main__':
    img = cv2.imread('raw_data/image.png')

    img_lines_file = 'picked_img_lines.txt'
    edge_lines_file = 'picked_3d_edge_lines.txt'
    img_lines, img_lines_normals = get_img_lines_normal(img_lines_file)
    lines, lines_dirs = get_3d_lines_dir(edge_lines_file)

    #show pairs
    # w_t_c = np.array([0.0, 0.0, 0.0])
    # w_R_c = np.array([  [0, 0, 1],
    #                     [-1,0, 0],
    #                     [0,-1, 0]])
    # for idx,l in enumerate(lines):
    #     img_temp = img.copy()
    #     pnt_a = project_3d_pt_on_img(P, w_R_c, w_t_c, l[0].flatten())
    #     pnt_b = project_3d_pt_on_img(P, w_R_c, w_t_c, l[1].flatten())
    #     cv2.circle(img_temp, (int(pnt_a[0,0]), int(pnt_a[1,0])), 6, [255, 0, 0], -1)
    #     cv2.circle(img_temp, (int(pnt_b[0,0]), int(pnt_b[1,0])), 6, [255, 0, 0], -1)
    #     img_temp = cv2.line(img_temp, tuple(pnt_a), tuple(pnt_b), (0,0,255), 2)

    #     pnt_a = np.array(img_lines[idx][0].flatten()).reshape(2,1)
    #     pnt_b = np.array(img_lines[idx][1].flatten()).reshape(2,1)
    #     cv2.circle(img_temp, (int(pnt_a[0,0]), int(pnt_a[1,0])), 6, [0, 255, 0], -1)
    #     cv2.circle(img_temp, (int(pnt_b[0,0]), int(pnt_b[1,0])), 6, [0, 255, 0], -1)
    #     img_temp = cv2.line(img_temp, tuple(pnt_a), tuple(pnt_b), (255,0,255), 2)

    #     cv2.imshow('pair_img', cv2.resize(img_temp, (900,600)))
    #     cv2.waitKey(0)

    q_init = np.array([0.5, -0.5, 0.5, -0.5])
    T_init = np.array([0.0, 0.0, 0.0])

    w_q_c = autograd.Variable(torch.from_numpy(q_init.astype(dtype=float)),requires_grad=True)
    w_T_c = autograd.Variable(torch.from_numpy(T_init.astype(dtype=float)),requires_grad=True)
    D = autograd.Variable(torch.from_numpy(lines_dirs.astype(dtype=float)))
    N = autograd.Variable(torch.from_numpy(img_lines_normals.astype(dtype=float)))

    w_R_c = kornia.quaternion_to_rotation_matrix(w_q_c.data).numpy()

    # visualize initial conditions
    for line in img_lines:
        pnt_a = np.array(line[0].flatten()).reshape(2,1)
        pnt_b = np.array(line[1].flatten()).reshape(2,1)
        cv2.circle(img, (int(pnt_a[0,0]), int(pnt_a[1,0])), 6, [0, 255, 0], -1)
        cv2.circle(img, (int(pnt_b[0,0]), int(pnt_b[1,0])), 6, [0, 255, 0], -1)
        img = cv2.line(img, tuple(pnt_a), tuple(pnt_b), (255,0,255), 2)

    vis_pts(img, P, lines, w_R_c, w_T_c.data.numpy())

    pt_3d = []
    for line in lines:
        pt_3d.append(line[0])

    pts_3d = torch.from_numpy(np.array(pt_3d).T)
    rot_done = 0
    t_done = 0
    for i in range(2000000):
        total_rot = cost_function_rot(N, w_q_c, D)
        total_rot.backward()
        w_q_c.data -= .001 * w_q_c.grad.data
        w_q_c.data = kornia.normalize_quaternion(w_q_c.data)
        del_q = .001 * w_q_c.grad.data.numpy()
        w_q_c.grad.data.zero_()
        if (np.sqrt(np.dot(del_q, del_q)) < 1e-4):
            rot_done = 1

        w_q_c_cal = w_q_c.data
        total_trans = cost_function_trans(N, w_q_c_cal, w_T_c.view(3,1), pts_3d)
        total_trans.backward()
        w_T_c.data -= .01 * w_T_c.grad.data
        del_t = .01 * w_T_c.grad.data
        w_T_c.grad.data.zero_()

        if (np.sqrt(np.dot(del_t, del_t)) < 1e-4):
            t_done = 1

        if i%10 == 0:
            print(f'residual rot err: {total_rot}')
            print(f'residual trans err: {total_trans}')
            print(f'translation: {w_T_c.data}')
            print(f'del_t: {np.sqrt(np.dot(del_t, del_t))}')
            print(f'del_q: {np.sqrt(np.dot(del_q, del_q))}')

            t_final = w_T_c.data.numpy()
            w_R_c_final = kornia.quaternion_to_rotation_matrix(w_q_c.data).numpy()
            eu = Rotation.from_matrix(w_R_c_final).as_euler('xyz', degrees=True)
            print(f"Euler angles: {eu}\n")
            vis_pts(img.copy(), P, lines, w_R_c_final, t_final, itr_flag=True)

        if t_done and rot_done:
            print("\n\nDONE!!!!!!!!!!!!!!!!!!!\n\n")
            print(f"Total itrs: {i}")
            break

    print(f'residual rot err: {total_rot}')
    print(f'residual trans err: {total_trans}')

    w_R_c_final = kornia.quaternion_to_rotation_matrix(w_q_c.data).numpy()
    # print(f'R_final: \n{w_R_c_final}')
    eu = Rotation.from_matrix(w_R_c_final).as_euler('xyz', degrees=True)
    # eu_rad = Rotation.from_matrix(w_R_c_final.T).as_euler('xyz', degrees=False)
    print(f"Calculated Euler angles: {eu}")

    t_final = w_T_c.data.numpy()
    print(f"final pos: {t_final}")

    vis_pts(img,
            P,
            lines,
            w_R_c_final,
            t_final,
            itr_flag=True,
            wait_time=0)

    cv2.destroyAllWindows()