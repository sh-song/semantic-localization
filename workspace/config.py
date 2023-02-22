import numpy as np
class Config:
    K_left_color = np.array([[9.597910e+02, 0.000000e+00, 6.960217e+02], 
                            [0.000000e+00, 9.569251e+02, 2.241806e+02], 
                            [0.000000e+00, 0.000000e+00, 1.000000e+00]])
    dist_left_color = np.array([-3.691481e-01, 1.968681e-01, 1.353473e-03, 5.677587e-04, -6.770705e-02])

    K_right_color = np.array([[9.037596e+02, 0.000000e+00, 6.957519e+02], 
                            [0.000000e+00, 9.019653e+02, 2.242509e+02], 
                            [0.000000e+00, 0.000000e+00, 1.000000e+00]])
    dist_right_color = np.array([-3.639558e-01, 1.788651e-01, 6.029694e-04, -3.922424e-04, -5.382460e-02])



    save_path = 'output'

    camera_pose_VC = np.array([1, 0, 1.5, 0, np.deg2rad(90+40), np.deg2rad(0)]) 

    ## Image size

    IMG_H = 1080
    IMG_W = 1920

    plt_pose_IMG = np.array([IMG_H//2, IMG_W//2, 0, 0, np.deg2rad(180), np.deg2rad(-90)]) 
    cv_pose_IMG = np.array([IMG_H//2, IMG_W//2, 0, np.deg2rad(180), np.deg2rad(0), np.deg2rad(0)]) 


    base_lla = [37.3890294, 126.6487574, 12.651] 
    init_lla = [37.389059, 126.648787, 6.4]
    precision = '1.0'
    map_path = './data/maps/songdo'


    initial_vehicle_pose_WC = np.array([2.621263252100426, 3.2851581777834498, 0.39999861389485725, 0, 0, np.deg2rad(140)])
    test_scene_llas={'scene1': [37.395946364, 126.637618145, 13.659100372314455],
                    'scene2': [37.400976112, 126.632935096, 13.99969949951172],
                    'scene3': [37.402134501, 126.63204485799999, 14.002799215698243],
                    'base': [37.3890294, 126.6487574,6.0]}
