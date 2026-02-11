#!/usr/bin/env python3 
#camera_class.py

# -----------------------------------
# Se encarga de procesar las imagenes
# La clase devuelve:
#   Una imagen procesada (por ejemplo con contornos o marcas).
#   Un comando (cadena de texto) que se publica en /bebop/command.
#
# Este script define una clase llamada BebopCameraProcessor que procesa 
#  imágenes capturadas por la cámara de un dron Parrot Bebop.

# Su objetivo es detectar cuadrados naranjas usando dos métodos complementarios:
#   - YOLO (red neuronal entrenada) → detección general de objetos.
#   - OpenCV (procesamiento clásico de imágenes) → detección por color y forma.
# Después, combina los resultados de ambos métodos para:
#   - Dibujar los objetos detectados en la imagen,
#   - Determinar el centro del cuadrado más grande,
#   - Calcular si el cuadrado está a la izquierda, derecha o centro respecto al campo de visión,
# Y en base a eso, genera un comando de movimiento para el dron:
#   - "a" → moverse a la izquierda
#   - "d" → moverse a la derecha
#   - "w" → avanzar
#
#   Solamente usa realmente yolo porque la mascara para open cv de momento esta para color azul pero al detectar naranja,
#   no lo hace bien, por eso se opta por usar solo YOLO que funciona correctamente.
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

# Sube un nivel y entra a la carpeta 'classes'
project_root = os.path.abspath(os.path.join(current_dir, '..', 'classes'))

# Ruta al modelo YOLO entrenado (best.pt). To download the model, download from Github. 
model_path = os.path.join(project_root, 'models', 'best.pt')


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
    # PROCESAMIENTO PRINCIPAL
    # ---------------------------
    def process_image(self, cv_image):
        # Obtain hedght and widht to get center
        height, width = cv_image.shape[:2] # Obtiene dimensiones de la imagen
        self.center = (width // 2, height // 2) # Centro de la imagen

        # Detect squares using YOLO. 
        annotated_image, command_yolo = self.detect_with_yolo(cv_image.copy())

        # Detect squares using OpenCV
        processed_image, command_opencv = self.detect_square(cv_image.copy())

        # Combine both detections. Combina visualmente ambas detecciones con una ponderacion de 50% cada una
        combined_image = cv2.addWeighted(annotated_image, 0.5, processed_image, 0.5, 0)

        # Decied command due to detection (prioritize OpenCV). Prioriza el comando de OpenCV (más preciso en color) 
        if command_opencv:
            command = command_opencv 
        else:
            command = command_yolo

        return combined_image, command

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
        command = None

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
            cY = (y1 + y2) // 2
            cv2.circle(annotated_image, (cX, cY), 5, (0, 0, 255), -1) # Punto ROJO. Centro del cuadrado

            # Determina comando de movimiento según posición del cuadrado
            # Decide command based on position. 
            if cX < self.center[0] - 25:
                command = "a"  # Move to the left
            elif cX > self.center[0] + 25:
                command = "d"  # Move to the right
            else:
                command = "w"  # Move forward

            # Draw biggest square
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2) # VERDE
        else:
            print("Couln't find valid square with YOLO")

        # Draw image center. # Dibuja el centro de la imagen (referencia)
        cv2.circle(annotated_image, self.center, 5, (255, 0, 0), -1) # Punto AZUL

        return annotated_image, command



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
        command = None

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
                cY = int(M["m01"] / M["m00"])
                # Draw circle in the center
                cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1) # Punto Rojo

                # Decide command based on position
                if cX < self.center[0] - 25:
                    command = "a"  # Move left
                elif cX > self.center[0] + 25:
                    command = "d"  # Move right
                else:
                    command = "w"  # Go forward
            else:
                print("Could not obtain square's center")
        else:
            print("Couln't find valid square with OpenCV")

        # Obtain image center. Dibuja el centro de la imagen (referencia)
        cv2.circle(cv_image, self.center, 5, (255, 0, 0), -1) # Punto Azul

        return cv_image, command
