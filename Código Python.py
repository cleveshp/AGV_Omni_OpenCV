'''
Comando para executar o código-
python pose_estimation.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
'''


import numpy as np
import cv2
import sys
from utils import ARUCO_DICT
import argparse
import time
import math
import http.client
import json
import pip._vendor.requests 

# Endereço IP do ESP32
esp32_ip = "192.168.0.10"



# Verfica se a matris de rotação e valida.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calcula a matriz de rotacao angulos de euler

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

#--- 180 graus matriz de rotação em torno do eixo x
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0


# Endereço IP do ESP32
esp32_ip = "192.168.0.10"

font = cv2.FONT_HERSHEY_PLAIN



def pose_esitmation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    '''
    frame - Frame do video
    matrix_coefficients - Matriz de calibracao
    distortion_coefficients - Matriz de distorcoes
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()


    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
        cameraMatrix=matrix_coefficients,
        distCoeff=distortion_coefficients)

        # Verifica se o marcador foi encotrado
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estima a pose de cada marcador encontrado e retorna os valores rvec e tvec 
            ret = cv2.aruco.estimatePoseSingleMarkers(corners[i], 1.15, matrix_coefficients,
                                                                       distortion_coefficients)



             #-- captura o retorno
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

            #-- desenha o marcador na tela
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)

            #-- escreve a posicao do marcador
            str_position = "MARKER Position x=%4.2f  y=%4.2f  z=%4.2f"%(tvec[0], tvec[1], tvec[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)



            #-- obtem a matriz de rotação tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T

            #-- obterm a rotacao em torno de cada eixo
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

            #-- escreve as rotaçoes obtidas
            str_attitude = "MARKER Attitude r=%4.2f  p=%4.2f  y=%4.2f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                math.degrees(yaw_marker))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
	    #-- salva os valores de interesse
            coordenadaX=int(100*tvec[0])
            coordenadaY=int(100*tvec[1])
            angulo=int(math.degrees(yaw_marker))
            
            url = f"http://{esp32_ip}/?valor01={coordenadaX}&valor02={coordenadaY}&valor03={angulo}"
            # Enviar requisição GET para o servidor
            print(url)
            #response = pip._vendor.requests.get(url)


            #-- pega os valores em relacao a camera
            pos_camera = -R_tc*np.matrix(tvec).T


            str_position = "CAMERA Position x=%4.2f  y=%4.2f  z=%4.2f"%(pos_camera[0], pos_camera[1], pos_camera[2])
            cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            #-- pega a rotacao em relacao a camera
            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
            str_attitude = "CAMERA Attitude r=%4.2f  p=%4.2f  y=%4.2f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            #tvec vetor de translacao, rvec vetor de rotacao
            time.sleep(1.1)

    return frame

if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
    ap.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())

    
    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)

    aruco_dict_type = ARUCO_DICT[args["type"]]
    calibration_matrix_path = args["K_Matrix"]
    distortion_coefficients_path = args["D_Coeff"]
    
    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    video = cv2.VideoCapture(0)
    time.sleep(2.0)


    while True:
        ret, frame = video.read()

        if not ret:
            break
        
        output = pose_esitmation(frame, aruco_dict_type, k, d)

        cv2.imshow('Estimated Pose', output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()
