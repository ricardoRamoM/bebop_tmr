#!/usr/bin/env python3 
#window_orange_detector.py (ahora)
#camera_class.py (antes)

# -----------------------------------
# Clase de Percepción – Window Detection
#
# Este script define una clase encargada exclusivamente del procesamiento
# de imágenes provenientes de la cámara del dron Parrot Bebop.
#
# RESPONSABILIDAD:
#   - Detectar la ventana/cuadrado naranja en la imagen.
#   - Calcular información geométrica relevante.
#   - Devolver datos estructurados a la misión.
#
# IMPORTANTE:
#   Este nodo NO toma decisiones de movimiento.
#   No publica comandos de vuelo.
#   No contiene lógica de misión.
#
# FUNCIONAMIENTO:
#
# Utiliza YOLO (red neuronal entrenada) para detectar la ventana naranja.
# Aunque originalmente se contemplaba un método adicional con OpenCV
# basado en segmentación por color y forma, actualmente se utiliza
# únicamente YOLO debido a que ofrece mayor robustez y precisión
# en la detección del objeto.
#
# La clase procesa cada frame y devuelve:
#
#   - Imagen procesada con detecciones dibujadas.
#   - Booleano indicando si la ventana fue detectada.
#   - Centro del bounding box (cx, cy).
#   - Área del bounding box.
#   - Dimensiones del frame.
#
# SALIDA:
#   La información se publica como datos estructurados
#   (por ejemplo en un mensaje custom o diccionario),
#   que luego es utilizada por el nodo de misión.
#
# ARQUITECTURA (Nueva Estructura):
#
#   Percepción  →  SOLO detecta
#   Misión      →  Decide qué hacer con esa detección
#   Control     →  Ejecuta los movimientos del dron
#
# Esta separación permite:
#   - Escalar a nuevas misiones fácilmente.
#   - Cambiar lógica sin tocar visión.
#   - Cumplir arquitectura modular estilo competencia (TMR-ready).
# -----------------------------------

import os
import cv2
import numpy as np 
from ultralytics import YOLO # Librería del modelo YOLOv8

# ---------------------------
# CONFIGURACIÓN DE RUTAS
# ---------------------------

# Obtiene el directorio actual del archivo
current_dir = os.path.dirname(os.path.abspath(__file__))

# Ruta al modelo YOLO entrenado (best.pt). To download the model, download from Github. 
model_path = os.path.join(current_dir, 'models', 'best.pt')

print("Loading model YOLO from:", model_path)

if not os.path.exists(model_path):
    raise FileNotFoundError(f"Model was not found in the directory: {model_path}")

# ---------------------------
# CLASE PRINCIPAL
# ---------------------------
class BebopCameraProcessor:
    def __init__(self):
        """Inicializa la clase y carga el modelo YOLO."""
        try:
            print(f"Loading model YOLO from: {model_path}")

            # Verifica que el archivo del modelo exista
            if not os.path.exists(model_path):
                raise FileNotFoundError(f"Model was not found in the directory: {model_path}")
            
            # Carga el modelo YOLO
            self.model = YOLO(model_path)
        except Exception as e:
            print(f"Couldn't load YOLO: {e}")
            raise e # Re-lanza el error para detener la ejecución

    # ---------------------------
    # PROCESAMIENTO PRINCIPAL que se manda 
    # ---------------------------
    def process_image(self, cv_image):
        # Obtain hedght and widht to get center
        height, width = cv_image.shape[:2] # Obtiene dimensiones de la imagen
        self.center = (width // 2, height // 2) # Centro de la imagen

        # Detect squares using YOLO. 
        annotated_image, yolo_detected, yolo_cx = self.detect_with_yolo(cv_image.copy())

        # Detect squares using OpenCV
        processed_image, cv_detected, cv_cx = self.detect_square(cv_image.copy())

        # Combine both detections. Combina visualmente ambas detecciones con una ponderacion de 50% cada una
        combined_image = cv2.addWeighted(annotated_image, 0.5, processed_image, 0.5, 0)

        # Priorizar OpenCV si detecta
        if cv_detected:
            detected = True
            cx = cv_cx
        elif yolo_detected:
            detected = True
            cx = yolo_cx
        else:
            detected = False
            cx = None

        return combined_image, {
            "detected": detected,
            "cx": cx,
            "center_x": self.center[0]
        }

    # ---------------------------
    # DETECCIÓN CON YOLO
    # ---------------------------
    def detect_with_yolo(self, cv_image):
        """Detecta cuadrados usando el modelo YOLO."""
        
        # Ejecuta predicción con el modelo
        # Process image with YOLO
        results = self.model.predict(source=cv_image)
        annotated_image = results[0].plot() # Dibuja resultados detectados

        # Variables
        biggest_area = 0
        square_coords = None
        detected = False
        cx = None

        boxes = results[0].boxes  # Obtain detected boxes  # Obtiene las cajas detectadas
        
        if boxes is not None and boxes.xyxy is not None:
            # Convert coordinates to numpy array
            xyxy = boxes.xyxy.cpu().numpy()  # (n, 4) # Convierte a arreglo NumPy (n, 4)
            
            # Recorre cada caja detectada
            for box_coords in xyxy:
                x1, y1, x2, y2 = box_coords
                width_box = x2 - x1
                height_box = y2 - y1
                if height_box == 0:
                    continue  # Do not divide by 0

                # Calcula relación de aspecto para verificar si es cuadrado
                aspect_ratio = width_box / float(height_box)
                if 0.9 <= aspect_ratio <= 1.1:  # Verify if it is an square
                    area = width_box * height_box
                    if area > biggest_area:
                        biggest_area = area
                        square_coords = [int(x1), int(y1), int(x2), int(y2)]

        # If square found then decide command and draw square. # Si encontró un cuadrado
        if square_coords:
            x1, y1, x2, y2 = square_coords
            cX = (x1 + x2) // 2
            cx = cX
            cY = (y1 + y2) // 2
            cv2.circle(annotated_image, (cX, cY), 5, (0, 0, 255), -1) # Punto ROJO. Centro del cuadrado

            # Draw biggest square
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2) # VERDE

            detected = True

        else:
            print("Couln't find valid square with YOLO")

        # Draw image center. # Dibuja el centro de la imagen (referencia)
        cv2.circle(annotated_image, self.center, 5, (255, 0, 0), -1) # Punto AZUL

        return annotated_image, detected, cx



    def detect_square(self, cv_image):
        # No usa el naranja porque no funciona bien por la luz y otros factores
        # HSV range for ORANGE color
        #lower_orange = np.array([5, 100, 100])
        #upper_orange = np.array([20, 255, 255])
        # Mask for only orange parts
        #mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

        """Detecta cuadrados azules usando procesamiento clásico de imagen."""
        
        # Convierte la imagen de BGR a HSV
        # From BGR 2 HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Obtain upper and lower blue limits for HSV. Define rangos de color azul en HSV
        lower_blue = np.array([110, 150, 150])
        upper_blue = np.array([130, 255, 255])

        # Mask for only blue parts. Crea una máscara solo para la región azul
        mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Apply mask to original image. Aplica la máscara a la imagen original
        blue_only = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # BGR 2 Gray and blur. Convierte a escala de grises y suaviza la imagen
        gray = cv2.cvtColor(blue_only, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Obtain canny edges. Detecta bordes con Canny
        edges = cv2.Canny(blurred, 50, 150)

        # Dilate borders to make them thicker. Engrosa bordes para facilitar detección de contornos
        kernel = np.ones((5, 5), np.uint8)
        thick_edges = cv2.dilate(edges, kernel, iterations=1)

        # Find contours within borders. Busca contornos externos
        contours, _ = cv2.findContours(thick_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Variables
        biggest_area = 0
        square_contour = None
        detected = False
        cx = None

        # Iterate over the contours to find squares. Recorre todos los contornos detectados
        for contour in contours:
            # Aproximate contours to polygons
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Verify if the polygon has 4 vertices and is convex. Verifica si el contorno tiene 4 lados y es convexo
            if len(approx) == 4 and cv2.isContourConvex(approx):
                area = cv2.contourArea(approx)
                if area > biggest_area:
                    biggest_area = area
                    square_contour = approx

        # If square found, then: 
        if square_contour is not None:
            # Draw square contour
            cv2.drawContours(cv_image, [square_contour], -1, (0, 255,0), 2) # VERDE 

            # Obtain center square. Calcula el centro geométrico del cuadrado
            M = cv2.moments(square_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cx = cX
                cY = int(M["m01"] / M["m00"])
                # Draw circle in the center
                cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1) # Punto Rojo
                detected = True

            else:
                print("Could not obtain square's center")
        else:
            print("Couln't find valid square with OpenCV")

        # Obtain image center. Dibuja el centro de la imagen (referencia)
        cv2.circle(cv_image, self.center, 5, (255, 0, 0), -1) # Punto Azul

        return cv_image, detected, cx
    
# =====================================================
# TEST NODE (solo para validación independiente)
# =====================================================

#if __name__ == "__main__":
#    import rospy
#    from sensor_msgs.msg import Image
#    from cv_bridge import CvBridge
#
#    rospy.init_node("window_orange_detector_test")
#
#    bridge = CvBridge()
#    detector = BebopCameraProcessor()
#
#   def image_callback(msg):
#       try:
#            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
#        except Exception as e:
#            rospy.logerr(f"CvBridge error: {e}")
#            return
#
#        processed_image, data = detector.process_image(frame)
#
#        cv2.imshow("Window Detector Test", processed_image)
#        cv2.waitKey(1)
#        if data["detected"]:
#            rospy.loginfo(f"Detected | cx: {data['cx']}")
#
#    rospy.Subscriber("/bebop/image_raw", Image, image_callback)
#
#    rospy.loginfo("Window detector test running...")
#    rospy.spin()