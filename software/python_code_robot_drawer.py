import sys
import time
import xml.etree.ElementTree as ET
import serial
import random
import math
from PyQt5.QtCore import QPointF, Qt, QTimer
from PyQt5.QtGui import QPolygonF, QPen, QColor
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QFileDialog, QGraphicsScene, QGraphicsView,
                             QGraphicsPolygonItem, QFrame, QTableWidget, QTableWidgetItem,
                             QHeaderView, QMessageBox, QProgressBar, QComboBox)
from svgpathtools import svg2paths2


class TraceurApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Traceur")
        self.setGeometry(100, 100, 1200, 700)
        self.current_animation_step = 0
        self.is_paused = False
        self.arduino_connection = None
        self.polygons = []
        self.port_com = "COM3"  # Port par défaut
        self.initUI()

    def initUI(self):
        main_widget = QWidget(self)
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # Panneau de gauche (commandes)
        left_panel = QFrame()
        left_panel.setFrameShape(QFrame.StyledPanel)
        left_panel.setStyleSheet("background-color:#ADD8E6; border-radius: 10px;")
        left_layout = QVBoxLayout(left_panel)
        title_label = QLabel("Robot Traceur")
        title_label.setStyleSheet("font-size: 24px; font-weight: bold; color:#000000 ;")
        left_layout.addWidget(title_label)

        # Sélection du port COM
        port_layout = QHBoxLayout()
        port_label = QLabel("Port COM:")
        self.port_combo = QComboBox()
        self.port_combo.addItems([f"COM{i}" for i in range(1, 10)])
        self.port_combo.setCurrentText(self.port_com)
        self.port_combo.currentTextChanged.connect(self.update_port)
        port_layout.addWidget(port_label)
        port_layout.addWidget(self.port_combo)
        left_layout.addLayout(port_layout)

        self.load_button = QPushButton("Charger fichier SVG")
        self.load_button.setStyleSheet(
            "QPushButton { background-color:#FFFFFF ; color: black; border-radius: 5px; padding: 10px; font-size: 16px; }")
        self.load_button.clicked.connect(self.load_svg_file)
        left_layout.addWidget(self.load_button)

        self.polygon_label = QLabel("Aucun polygone chargé")
        self.polygon_label.setWordWrap(True)
        left_layout.addWidget(self.polygon_label)

        self.draw_button = QPushButton("Démarrer le dessin")
        self.draw_button.setStyleSheet(
            "QPushButton { background-color:#FFFFFF ; color: black; border-radius: 5px; padding: 10px; font-size: 16px; }")
        self.draw_button.clicked.connect(self.start_drawing)
        left_layout.addWidget(self.draw_button)

        self.sa_button = QPushButton("Optimiser avec Recuit Simulé")
        self.sa_button.setStyleSheet(
            "QPushButton { background-color:#FFFFFF ; color: black; border-radius: 5px; padding: 10px; font-size: 16px; }")
        self.sa_button.clicked.connect(self.start_drawing_sa)
        left_layout.addWidget(self.sa_button)

        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        left_layout.addWidget(self.progress_bar)

        self.pause_button = QPushButton("Pause")
        self.pause_button.setStyleSheet(
            "QPushButton { background-color:#FFFFFF ; color: black; border-radius: 5px; padding: 10px; font-size: 16px; }")
        self.pause_button.clicked.connect(self.pause_animation)
        self.pause_button.setVisible(False)
        left_layout.addWidget(self.pause_button)

        self.resume_button = QPushButton("Reprendre")
        self.resume_button.setStyleSheet(
            "QPushButton { background-color:#FFFFFF ; color: black; border-radius: 5px; padding: 10px; font-size: 16px; }")
        self.resume_button.clicked.connect(self.resume_animation)
        self.resume_button.setVisible(False)
        left_layout.addWidget(self.resume_button)

        self.gcode_button = QPushButton("Générer G-code")
        self.gcode_button.setStyleSheet(
            "QPushButton { background-color:#FFFFFF ; color: black; border-radius: 5px; padding: 10px; font-size: 16px; }")
        self.gcode_button.clicked.connect(self.generate_gcode)
        left_layout.addWidget(self.gcode_button)

        # Ajout d'un bouton pour tester la connexion Arduino
        self.test_arduino_button = QPushButton("Tester connexion Arduino")
        self.test_arduino_button.setStyleSheet(
            "QPushButton { background-color:#FFFFFF ; color: black; border-radius: 5px; padding: 10px; font-size: 16px; }")
        self.test_arduino_button.clicked.connect(self.test_arduino_connection)
        left_layout.addWidget(self.test_arduino_button)

        left_layout.addStretch()

        # Panneau de droite (affichage)
        right_panel = QFrame()
        right_layout = QVBoxLayout(right_panel)

        self.view = QGraphicsView()
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)
        right_layout.addWidget(self.view)

        self.table = QTableWidget()
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(["X", "Y"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        right_layout.addWidget(self.table)

        main_layout.addWidget(left_panel, 1)
        main_layout.addWidget(right_panel, 3)

    def update_port(self, new_port):
        """Met à jour le port COM sélectionné"""
        self.port_com = new_port
        if self.arduino_connection and self.arduino_connection.is_open:
            self.close_arduino_connection()

    def load_svg_file(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Ouvrir fichier SVG", "", "Fichiers SVG (*.svg)")
        if file_path:
            self.polygon_label.setText(f"Fichier chargé: {file_path}")
            self.polygons = self.extract_polygons_from_svg(file_path)
            if self.polygons:
                self.display_polygons()
                self.update_table()
            else:
                self.polygon_label.setText("Aucun polygone trouvé.")

    def extract_polygons_from_svg(self, file_path):
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            polygons = []
            for element in root.findall(".//{http://www.w3.org/2000/svg}polygon"):
                points = element.attrib.get("points", "")
                polygon = [tuple(map(float, p.split(','))) for p in points.split()]
                polygons.append(polygon)

            if polygons:
                # Appliquer l'échelle A4 aux polygones extraits
                return self.scale_to_A4(polygons)
            return []
        except Exception as e:
            QMessageBox.critical(self, "Erreur", f"Erreur lecture SVG: {e}")
            return []

    def scale_to_A4(self, polygons, width_mm=210, height_mm=297, margin_mm=10):
        if not polygons:
            return []

        all_points = [pt for poly in polygons for pt in poly]
        if not all_points:
            return polygons

        xs, ys = zip(*all_points)
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        svg_width = max_x - min_x
        svg_height = max_y - min_y

        if svg_width == 0 or svg_height == 0:
            return polygons

        available_width = width_mm - 2 * margin_mm
        available_height = height_mm - 2 * margin_mm

        scale = min(available_width / svg_width, available_height / svg_height)

        offset_x = (width_mm - svg_width * scale) / 2
        offset_y = (height_mm - svg_height * scale) / 2

        scaled_polygons = []
        for poly in polygons:
            scaled_poly = [(
                (x - min_x) * scale + offset_x,
                (y - min_y) * scale + offset_y
            ) for x, y in poly]
            scaled_polygons.append(scaled_poly)

        return scaled_polygons

    def display_polygons(self):
        self.scene.clear()
        pen = QPen(QColor(0, 0, 0))
        pen.setWidth(2)
        for polygon in self.polygons:
            qpolygon = QPolygonF([QPointF(x, y) for x, y in polygon])
            item = QGraphicsPolygonItem(qpolygon)
            item.setPen(pen)
            self.scene.addItem(item)
        self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

    def update_table(self):
        self.table.setRowCount(0)
        for polygon in self.polygons:
            for x, y in polygon:
                row = self.table.rowCount()
                self.table.insertRow(row)
                self.table.setItem(row, 0, QTableWidgetItem(f"{x:.2f}"))
                self.table.setItem(row, 1, QTableWidgetItem(f"{y:.2f}"))

    def start_drawing(self):
        if not self.polygons:
            QMessageBox.warning(self, "Erreur", "Aucun polygone à dessiner.")
            return

        # Initialisation de l'animation
        if not self.initialize_arduino_connection():
            return

        self.send_gcode_to_arduino("G90")  # Mode absolu
        # Attendre la confirmation de l'Arduino
        self.wait_for_arduino_response()

        self.display_optimized_polygons_with_animation()
        self.show_pause_resume_buttons()

    def start_drawing_sa(self):
        if not self.polygons:
            QMessageBox.warning(self, "Erreur", "Aucun polygone à optimiser.")
            return

        QMessageBox.information(self, "Optimisation", "Optimisation en cours...")
        optimized_polygons = self.simulate_sa_optimization(self.polygons)
        self.polygons = optimized_polygons
        self.display_optimized_polygons_with_animation()
        self.update_table()
        self.show_pause_resume_buttons()

    def simulate_sa_optimization(self, polygons):
        def total_distance(pts):
            return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1))

        optimized = []
        for polygon in polygons:
            if len(polygon) <= 2:
                optimized.append(polygon)
                continue
            current = polygon.copy()
            best = current[:]
            T, T_min, alpha = 1000.0, 0.1, 0.95
            while T > T_min:
                for _ in range(100):
                    i, j = random.sample(range(len(current)), 2)
                    current[i], current[j] = current[j], current[i]
                    delta = total_distance(current) - total_distance(best)
                    if delta < 0 or random.random() < math.exp(-delta / T):
                        best = current[:]
                T *= alpha
            optimized.append(best)
        return optimized

    def display_optimized_polygons_with_animation(self):
        self.scene.clear()
        self.current_animation_step = 0
        self.is_paused = False
        self.all_segments = []

        # Création de tous les segments pour l'animation
        for poly in self.polygons:
            if len(poly) < 2:
                continue
            self.all_segments.extend([(poly[i], poly[i + 1]) for i in range(len(poly) - 1)])
            self.all_segments.append((poly[-1], poly[0]))  # Ferme le polygone

        self.progress_bar.setMaximum(len(self.all_segments))
        self.progress_bar.setValue(0)
        self.progress_bar.setVisible(True)

        # Démarrer l'animation
        if self.all_segments:
            self.animate_next_segment()
        else:
            QMessageBox.warning(self, "Attention", "Aucun segment à animer")

    def animate_next_segment(self):
        if self.is_paused:
            return

        if self.current_animation_step >= len(self.all_segments):
            self.polygon_label.setText("Animation terminée.")
            self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
            self.progress_bar.setVisible(False)
            self.hide_pause_resume_buttons()
            # Terminer le dessin et lever le stylo
            self.send_gcode_to_arduino("G0 Z5")  # Lever le stylo
            self.wait_for_arduino_response()
            return

        # Animation du segment actuel
        start, end = self.all_segments[self.current_animation_step]
        pen = QPen(QColor(0, 0, 255), 2)
        self.scene.addLine(start[0], start[1], end[0], end[1], pen)
        self.progress_bar.setValue(self.current_animation_step + 1)

        # Premier segment : aller en position sans dessiner
        if self.current_animation_step == 0:
            # IMPORTANT: Envoyer Y comme coordonnée Z pour l'Arduino
            self.send_gcode_to_arduino(f"G0 X{start[0]:.2f} Y{start[1]:.2f}")  # Aller au point de départ
            self.wait_for_arduino_response()
            self.send_gcode_to_arduino("G0 Z0")  # Abaisser le stylo
            self.wait_for_arduino_response()

        # Dessiner le segment
        # IMPORTANT: Envoyer Y comme coordonnée Z pour l'Arduino
        self.send_gcode_to_arduino(f"G1 X{end[0]:.2f} Y{end[1]:.2f}")
        self.wait_for_arduino_response()

        self.current_animation_step += 1
        QTimer.singleShot(100, self.animate_next_segment)  # Animation plus rapide

    def pause_animation(self):
        self.is_paused = True
        self.pause_button.setVisible(False)
        self.resume_button.setVisible(True)
        # Actuellement, on ne fait que mettre en pause le code Python,
        # pas besoin d'envoyer de commande spécifique à l'Arduino

    def resume_animation(self):
        self.is_paused = False
        self.pause_button.setVisible(True)
        self.resume_button.setVisible(False)
        QTimer.singleShot(100, self.animate_next_segment)

    def convert_polygons_to_gcode(self, polygons):
        # MODIFIÉ pour adapter à l'Arduino qui utilise X et Z (au lieu de X et Y)
        gcode_lines = [
            "G21 ; Set units to millimeters",
            "G90 ; Use absolute coordinates",
            "G1 F1000 ; Set default feedrate",
            "G0 Z5 ; Pen up (safety start)"
        ]

        for i, poly in enumerate(polygons):
            if not poly:
                continue
            x0, y0 = poly[0]
            gcode_lines.append(f"\n; --- Polygon {i + 1} ---")
            # Utiliser Y dans la commande pour l'Arduino qui l'interprète comme Z
            gcode_lines.append(f"G0 X{x0:.2f} Y{y0:.2f} ; Move to start point")
            gcode_lines.append("G0 Z0 ; Pen down")

            for x, y in poly[1:]:
                # Utiliser Y dans la commande pour l'Arduino qui l'interprète comme Z
                gcode_lines.append(f"G1 X{x:.2f} Y{y:.2f}")

            # Utiliser Y dans la commande pour l'Arduino qui l'interprète comme Z
            gcode_lines.append(f"G1 X{x0:.2f} Y{y0:.2f} ; Close polygon")
            gcode_lines.append("G0 Z5 ; Pen up")

        # Utiliser Y dans la commande pour l'Arduino qui l'interprète comme Z
        gcode_lines.append("\nG0 X0 Y0 ; Return to origin")
        gcode_lines.append("M2 ; End of program")

        return '\n'.join(gcode_lines)

    def generate_gcode(self):
        if not self.polygons:
            QMessageBox.warning(self, "Erreur", "Aucun polygone chargé.")
            return

        gcode = self.convert_polygons_to_gcode(self.polygons)

        # Affichage dans une fenêtre détaillée
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("G-code généré")
        msg_box.setText("Le G-code a été généré avec succès.")
        msg_box.setDetailedText(gcode)
        msg_box.setStandardButtons(QMessageBox.Save | QMessageBox.Ok)
        ret = msg_box.exec_()

        # Si l'utilisateur clique sur "Enregistrer"
        if ret == QMessageBox.Save:
            file_path, _ = QFileDialog.getSaveFileName(self, "Enregistrer le G-code", "", "Fichiers G-code (*.gcode)")
            if file_path:
                try:
                    with open(file_path, 'w') as f:
                        f.write(gcode)
                    QMessageBox.information(self, "Succès", f"G-code enregistré dans :\n{file_path}")
                except Exception as e:
                    QMessageBox.critical(self, "Erreur", f"Erreur lors de l'enregistrement : {e}")

    def initialize_arduino_connection(self):
        """Initialise la connexion avec l'Arduino"""
        try:
            if self.arduino_connection and self.arduino_connection.is_open:
                self.arduino_connection.close()

            self.arduino_connection = serial.Serial(self.port_com, 115200, timeout=2)
            time.sleep(2)  # Attendre que l'Arduino soit prêt
            return True
        except serial.SerialException as e:
            QMessageBox.critical(self, "Erreur",
                                 f"Impossible de se connecter à l'Arduino sur le port {self.port_com}: {e}")
            self.arduino_connection = None
            return False

    def close_arduino_connection(self):
        """Ferme la connexion avec l'Arduino"""
        if self.arduino_connection and self.arduino_connection.is_open:
            try:
                self.arduino_connection.close()
            except Exception as e:
                print(f"Erreur lors de la fermeture de la connexion: {e}")

    def send_gcode_to_arduino(self, gcode):
        """Envoie une commande G-code à l'Arduino"""
        try:
            if not self.arduino_connection or not self.arduino_connection.is_open:
                if not self.initialize_arduino_connection():
                    return False

            # Ajouter un retour à la ligne à la fin du G-code
            gcode_with_newline = gcode + '\n'
            self.arduino_connection.write(gcode_with_newline.encode('utf-8'))

            # Debug: afficher le G-code envoyé
            print(f"G-code envoyé: {gcode}")
            return True

        except Exception as e:
            print(f"Erreur lors de l'envoi de la commande G-code: {e}")
            self.close_arduino_connection()
            return False

    def wait_for_arduino_response(self, timeout=5.0):
        """Attend une réponse de l'Arduino avec un timeout"""
        if not self.arduino_connection or not self.arduino_connection.is_open:
            return False

        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.arduino_connection.in_waiting > 0:
                response = self.arduino_connection.readline().decode('utf-8').strip()
                print(f"Réponse Arduino: {response}")
                # Vérifier si la réponse commence par "ok"
                if response.startswith("ok") or response.startswith("X:") or response.startswith("Y:"):
                    return True
                elif "erreur" in response.lower():
                    QMessageBox.warning(self, "Erreur Arduino", f"L'Arduino a signalé une erreur: {response}")
                    return False
            time.sleep(0.1)

        QMessageBox.warning(self, "Timeout", "L'Arduino n'a pas répondu dans le délai imparti")
        return False

    def test_arduino_connection(self):
        """Teste la connexion avec l'Arduino"""
        if self.initialize_arduino_connection():
            try:
                # Envoyer un G-code simple qui devrait être reconnu par l'Arduino
                self.send_gcode_to_arduino("G90")  # Mode absolu

                # Attendre une réponse avec un timeout
                if self.wait_for_arduino_response(2.0):
                    QMessageBox.information(self, "Test Réussi", "Communication établie avec l'Arduino.")
                else:
                    # Si pas de réponse à G90, essayons avec M114
                    self.send_gcode_to_arduino("M114")  # Demander la position actuelle
                    if self.wait_for_arduino_response(2.0):
                        QMessageBox.information(self, "Test Réussi", "Communication établie avec l'Arduino.")
                    else:
                        QMessageBox.warning(self, "Test", "Connexion établie mais pas de réponse de l'Arduino")

            except Exception as e:
                QMessageBox.warning(self, "Test", f"Connexion établie mais erreur de lecture: {e}")
            finally:
                # Ne pas fermer la connexion si le test est réussi
                pass
        else:
            QMessageBox.critical(self, "Erreur", f"Impossible de se connecter à l'Arduino sur le port {self.port_com}")

    def show_pause_resume_buttons(self):
        self.pause_button.setVisible(True)
        self.resume_button.setVisible(False)

    def hide_pause_resume_buttons(self):
        self.pause_button.setVisible(False)
        self.resume_button.setVisible(False)

    def closeEvent(self, event):
        """Appelé lorsque l'application se ferme"""
        self.close_arduino_connection()
        super().closeEvent(event)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = TraceurApp()
    window.show()
    sys.exit(app.exec_())