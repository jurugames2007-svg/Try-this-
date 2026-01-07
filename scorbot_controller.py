import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import time
import threading
from collections import deque
import serial.tools.list_ports
import pickle

class ScorbotController:
    def __init__(self, root):
        self.root = root
        self.root.title("Scorbot ER XI Controller")
        self.root.geometry("800x700")

        # Serial connection
        self.serial_conn = None
        self.com_port = "COM1"  # Default
        self.baud_rate = 9600

        # Estado del Robot
        self.recording = False
        self.sequence = deque()
        self.playing = False
        self.homed = False

        # --- DICCIONARIOS DE POSICIÓN ---
        # Definimos joints primero para evitar errores en la GUI
        self.joints = {
            'base': 0, 'shoulder': 0, 'elbow': 0, 
            'wrist': 0, 'gripper': 0
        }
        
        self.cartesian_pos = {
            'x': 200, 'y': 0, 'z': 150, 
            'pitch': -90, 'roll': 0
        }

        # Límites de seguridad (Grados)
        self.joint_limits = {
            'base': (-150, 150), 'shoulder': (-90, 90),
            'elbow': (-90, 90), 'wrist': (-180, 180),
            'gripper': (0, 100)
        }

        # Límites cartesianos aproximados (basado en Scorbot ER XI)
        self.cartesian_limits = {
            'x': (-500, 500), 'y': (-500, 500), 'z': (0, 600),
            'pitch': (-180, 180), 'roll': (-180, 180)
        }

        # Longitudes del brazo (mm) para cinemática
        self.L1 = 200  # Base to shoulder
        self.L2 = 200  # Shoulder to elbow
        self.L3 = 200  # Elbow to wrist
        self.L4 = 100  # Wrist to tool

        # Variables de Tkinter
        self.joint_vars = {j: tk.StringVar(value=str(p)) for j, p in self.joints.items()}
        self.cartesian_vars = {a: tk.StringVar(value=str(p)) for a, p in self.cartesian_pos.items()}
        self.cartesian_mode = tk.StringVar(value="MOVED")
        self.speed_var = tk.IntVar(value=50)

        # Log widget
        self.log_text = tk.Text(self.root, height=5, state=tk.DISABLED, bg="black", fg="green", font=("Courier", 10))
        self.log_text.pack(fill=tk.X, padx=10, pady=5)

        self.setup_gui()

        # Protocol for closing window
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def is_reachable(self, x, y, z, pitch, roll):
        """Validación básica de alcance cartesiano"""
        # Verificar límites individuales
        if not (self.cartesian_limits['x'][0] <= x <= self.cartesian_limits['x'][1]):
            return False, "X fuera de límites"
        if not (self.cartesian_limits['y'][0] <= y <= self.cartesian_limits['y'][1]):
            return False, "Y fuera de límites"
        if not (self.cartesian_limits['z'][0] <= z <= self.cartesian_limits['z'][1]):
            return False, "Z fuera de límites"
        if not (self.cartesian_limits['pitch'][0] <= pitch <= self.cartesian_limits['pitch'][1]):
            return False, "Pitch fuera de límites"
        if not (self.cartesian_limits['roll'][0] <= roll <= self.cartesian_limits['roll'][1]):
            return False, "Roll fuera de límites"
        
        # Verificar distancia horizontal (radio máximo aproximado 500mm)
        horizontal_distance = (x**2 + y**2)**0.5
        if horizontal_distance > 500:
            return False, f"Distancia horizontal {horizontal_distance:.1f}mm excede el alcance máximo"
        
        return True, "Posición alcanzable"
    def inverse_kinematics(self, x, y, z, pitch, roll):
        """Calcula posiciones de joints para coordenadas cartesianas usando geometría analítica (aproximada)"""
        import math
        
        # Posición de la tool: ajustar por L4 en dirección de pitch/roll (simplificado)
        # Para simplificar, asumir que pitch/roll afectan solo wrist, tool en (x,y,z)
        x_w = x
        y_w = y
        z_w = z
        
        # Base angle (theta1)
        theta1 = math.atan2(y_w, x_w)  # Radianes
        
        # Distancia horizontal desde base
        r = math.sqrt(x_w**2 + y_w**2)
        
        # Altura desde base
        h = z_w - self.L1
        
        # Distancia desde shoulder a wrist en plano sagital
        d = math.sqrt(r**2 + h**2)
        
        # Verificar alcance
        if d > self.L2 + self.L3 or d < abs(self.L2 - self.L3):
            return None, "Posición fuera de alcance"
        
        # Ángulos shoulder (theta2) y elbow (theta3) usando ley de cosenos
        cos_theta3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        theta3 = math.acos(max(-1, min(1, cos_theta3)))
        
        # Para theta2
        alpha = math.atan2(h, r)
        beta = math.asin((self.L3 * math.sin(theta3)) / d)
        theta2 = alpha - beta
        
        # Wrist (theta4) para pitch (simplificado, asumir pitch afecta wrist)
        theta4 = math.radians(pitch)  # Convertir a radianes
        
        # Gripper no cambia
        theta5 = 0
        
        # Convertir a grados
        joints_calc = {
            'base': math.degrees(theta1),
            'shoulder': math.degrees(theta2),
            'elbow': math.degrees(theta3),
            'wrist': math.degrees(theta4),
            'gripper': 0
        }
        
        # Verificar límites
        for joint, pos in joints_calc.items():
            min_l, max_l = self.joint_limits[joint]
            if not (min_l <= pos <= max_l):
                return None, f"Joint {joint} fuera de límites: {pos}"
        
        return joints_calc, "Cálculo exitoso"
    def log_message(self, message):
        """Agrega un mensaje al log de comunicación"""
        self.log_text.config(state=tk.NORMAL)
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)  # Scroll to end
        self.log_text.config(state=tk.DISABLED)

    def get_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def setup_gui(self):
        # Emergency Stop Button
        self.stop_btn = tk.Button(self.root, text="EMERGENCY STOP", bg="red", fg="white", 
                                  font=("Arial", 14, "bold"), command=self.emergency_stop)
        self.stop_btn.pack(fill=tk.X, padx=10, pady=5)

        # Notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        # Tab 1: Manual Control
        self.manual_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.manual_tab, text="Manual Control")
        self.setup_manual_control()

        # Tab 2: Sequence Recording
        self.sequence_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.sequence_tab, text="Sequence Control")
        self.setup_sequence_control()

        # Tab 3: Settings
        self.settings_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.settings_tab, text="Settings")
        self.setup_settings()

        # Tab 4: Cartesian Control
        self.cartesian_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.cartesian_tab, text="Cartesian Control")
        self.setup_cartesian_control()

    def setup_manual_control(self):
        # Joint controls
        joints_frame = ttk.LabelFrame(self.manual_tab, text="Joint Control")
        joints_frame.pack(pady=10, padx=10, fill=tk.X)

        for joint, pos in self.joints.items():
            frame = ttk.Frame(joints_frame)
            frame.pack(fill=tk.X, pady=5)

            ttk.Label(frame, text=f"{joint.capitalize()}: ").pack(side=tk.LEFT)
            if joint == 'gripper':
                ttk.Button(frame, text="Open", command=lambda j=joint: self.set_gripper(j, 100)).pack(side=tk.LEFT)
                ttk.Button(frame, text="Close", command=lambda j=joint: self.set_gripper(j, 0)).pack(side=tk.LEFT)
            else:
                ttk.Button(frame, text="-", command=lambda j=joint: self.move_joint(j, -5)).pack(side=tk.LEFT)
                ttk.Button(frame, text="+", command=lambda j=joint: self.move_joint(j, 5)).pack(side=tk.LEFT)
            ttk.Label(frame, textvariable=self.joint_vars[joint]).pack(side=tk.LEFT, padx=10)

        # Speed control
        speed_frame = ttk.LabelFrame(self.manual_tab, text="Speed Control")
        speed_frame.pack(pady=10, padx=10, fill=tk.X)

        self.speed_var = tk.IntVar(value=50)
        ttk.Scale(speed_frame, from_=1, to=100, variable=self.speed_var, orient=tk.HORIZONTAL).pack(fill=tk.X)

        # Connect/Disconnect buttons
        button_frame = ttk.Frame(self.manual_tab)
        button_frame.pack(pady=10)
        ttk.Button(button_frame, text="Connect", command=self.connect_serial).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Disconnect", command=self.disconnect_serial).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Home", command=self.home).pack(side=tk.LEFT, padx=5)

    def setup_sequence_control(self):
        # Recording controls
        record_frame = ttk.LabelFrame(self.sequence_tab, text="Recording")
        record_frame.pack(pady=10, padx=10, fill=tk.X)

        ttk.Button(record_frame, text="Start Recording", command=self.start_recording).pack(side=tk.LEFT, padx=5)
        ttk.Button(record_frame, text="Stop Recording", command=self.stop_recording).pack(side=tk.LEFT, padx=5)
        ttk.Button(record_frame, text="Clear Sequence", command=self.clear_sequence).pack(side=tk.LEFT, padx=5)
        ttk.Button(record_frame, text="Save Sequence", command=self.save_sequence).pack(side=tk.LEFT, padx=5)
        ttk.Button(record_frame, text="Load Sequence", command=self.load_sequence).pack(side=tk.LEFT, padx=5)

        # Playback controls
        play_frame = ttk.LabelFrame(self.sequence_tab, text="Playback")
        play_frame.pack(pady=10, padx=10, fill=tk.X)

        ttk.Button(play_frame, text="Play Sequence", command=self.play_sequence).pack(side=tk.LEFT, padx=5)
        ttk.Button(play_frame, text="Stop Playback", command=self.stop_playback).pack(side=tk.LEFT, padx=5)

        # Sequence display
        self.sequence_text = tk.Text(self.sequence_tab, height=10, state=tk.DISABLED)
        self.sequence_text.pack(pady=10, padx=10, fill=tk.BOTH, expand=True)

    def setup_settings(self):
        # COM Port selection
        com_frame = ttk.LabelFrame(self.settings_tab, text="COM Port")
        com_frame.pack(pady=10, padx=10, fill=tk.X)

        self.com_var = tk.StringVar(value=self.com_port)
        self.com_combo = ttk.Combobox(com_frame, textvariable=self.com_var, values=self.get_ports())
        self.com_combo.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(com_frame, text="Refresh", command=self.refresh_ports).pack(side=tk.RIGHT)

        # Baud rate
        baud_frame = ttk.LabelFrame(self.settings_tab, text="Baud Rate")
        baud_frame.pack(pady=10, padx=10, fill=tk.X)

        self.baud_var = tk.IntVar(value=self.baud_rate)
        ttk.Combobox(baud_frame, textvariable=self.baud_var, values=[9600, 19200, 38400, 57600, 115200]).pack(fill=tk.X)

        # Diagnostic Test
        diag_frame = ttk.LabelFrame(self.settings_tab, text="Diagnostics")
        diag_frame.pack(pady=10, padx=10, fill=tk.X)

        ttk.Button(diag_frame, text="Run Diagnostic Test", command=self.calibrate_system).pack(fill=tk.X)

    def setup_cartesian_control(self):
        # Movement mode selection
        mode_frame = ttk.LabelFrame(self.cartesian_tab, text="Movement Mode")
        mode_frame.pack(pady=10, padx=10, fill=tk.X)
        ttk.Radiobutton(mode_frame, text="MOVED (Direct)", variable=self.cartesian_mode, value="MOVED").pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(mode_frame, text="MOVEL (Linear)", variable=self.cartesian_mode, value="MOVEL").pack(side=tk.LEFT, padx=5)

        cart_frame = ttk.LabelFrame(self.cartesian_tab, text="Control Cartesiano (X, Y, Z, Pitch, Roll)")
        cart_frame.pack(pady=10, padx=10, fill=tk.X)

        for axis in ['x', 'y', 'z', 'pitch', 'roll']:
            frame = ttk.Frame(cart_frame)
            frame.pack(fill=tk.X, pady=2)
            ttk.Label(frame, text=f"Eje {axis.upper()}: ", width=10).pack(side=tk.LEFT)
            ttk.Button(frame, text="-10", command=lambda a=axis: self.moved(a, -10)).pack(side=tk.LEFT)
            ttk.Label(frame, textvariable=self.cartesian_vars[axis]).pack(side=tk.LEFT, padx=10)
            ttk.Button(frame, text="+10", command=lambda a=axis: self.moved(a, 10)).pack(side=tk.LEFT)

        # Speed control for cartesian
        speed_frame = ttk.LabelFrame(self.cartesian_tab, text="Speed Control")
        speed_frame.pack(pady=10, padx=10, fill=tk.X)

        ttk.Scale(speed_frame, from_=1, to=100, variable=self.speed_var, orient=tk.HORIZONTAL).pack(fill=tk.X)

    def refresh_ports(self):
        self.com_combo['values'] = self.get_ports()

    def connect_serial(self):
        port = self.com_var.get()
        if port not in self.get_ports():
            messagebox.showerror("Error", f"Port {port} not available")
            return
        try:
            self.serial_conn = serial.Serial(port, self.baud_var.get(), timeout=1)
            messagebox.showinfo("Success", f"Connected to {port}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to connect: {str(e)}")

    def disconnect_serial(self):
        if self.serial_conn:
            self.serial_conn.close()
            self.serial_conn = None
            messagebox.showinfo("Success", "Disconnected")

    def home(self):
        if not self.serial_conn:
            messagebox.showwarning("Warning", "Not connected to robot")
            return
        # Send home command
        self.serial_conn.write(b"HOME\n")
        self.log_message("SENT: HOME")
        # Wait for response
        response = self.serial_conn.readline().decode().strip()
        self.log_message(f"RESP: {response}")
        # Reset internal positions to 0
        for joint in self.joints:
            self.joints[joint] = 0
            self.joint_vars[joint].set("0")
        # Reset cartesian positions
        self.cartesian_pos = {'x': 200, 'y': 0, 'z': 150, 'pitch': -90, 'roll': 0}
        for axis, pos in self.cartesian_pos.items():
            self.cartesian_vars[axis].set(str(pos))
        self.homed = True
        messagebox.showinfo("Home", "Homing command sent and positions reset to 0")

    def move_joint(self, joint, delta):
        if not self.homed:
            messagebox.showwarning("Homing Required", "Please perform homing before moving joints.")
            return
        
        # Limit delta to prevent abrupt movements
        if abs(delta) > 10:
            messagebox.showwarning("Delta Limit", "Movement delta too large. Max ±10 allowed.")
            return
        
        # 1. Calcular nueva posición
        new_pos = self.joints[joint] + delta
        
        # 2. Verificar límites de seguridad
        min_lim, max_lim = self.joint_limits[joint]
        if not (min_lim <= new_pos <= max_lim):
            messagebox.showwarning("Límite alcanzado", f"¡Límite de {joint.capitalize()} alcanzado!")
            return

        # 3. Actualizar estado interno y variable de la GUI
        self.joints[joint] = new_pos
        self.joint_vars[joint].set(str(new_pos))

        # 4. Grabar si el modo de grabación está activo
        speed = self.speed_var.get()
        if self.recording:
            self.sequence.append((joint, new_pos, speed))
            self.update_sequence_display()

        # 5. Enviar comando al robot vía Serial
        if self.serial_conn:
            # Formato típico ACL: MOVEA <ARTICULACIÓN> <POSICIÓN>
            # Ajusta el formato según tu firmware específico
            command = f"MOVEA {joint.upper()} {new_pos} SPEED={speed}\n"
            try:
                self.serial_conn.write(command.encode())
                self.log_message(f"SENT: {command.strip()}")
                # Wait for response
                response = self.serial_conn.readline().decode().strip()
                self.log_message(f"RESP: {response}")
            except Exception as e:
                self.log_message(f"ERROR: {e}")
                messagebox.showerror("Error Serial", f"No se pudo enviar: {e}")

    def set_gripper(self, joint, pos):
        if not self.homed:
            messagebox.showwarning("Homing Required", "Please perform homing before moving joints.")
            return
        
        # Set gripper to open (100) or close (0)
        self.joints[joint] = pos
        self.joint_vars[joint].set(str(pos))
        speed = self.speed_var.get()
        if self.recording:
            self.sequence.append((joint, pos, speed))
            self.update_sequence_display()
        if self.serial_conn:
            command = f"MOVEA {joint.upper()} {pos} SPEED={speed}\n"
            try:
                self.serial_conn.write(command.encode())
                self.log_message(f"SENT: {command.strip()}")
                # Wait for response
                response = self.serial_conn.readline().decode().strip()
                self.log_message(f"RESP: {response}")
            except Exception as e:
                self.log_message(f"ERROR: {e}")
                messagebox.showerror("Error Serial", f"No se pudo enviar: {e}")

    def moved(self, axis, delta):
        """Maneja movimientos MOVED (Directo) y MOVEL (Lineal)"""
        if not self.homed:
            messagebox.showwarning("Homing Required", "Realice el Homing antes de mover en modo cartesiano.")
            return

        # 1. Actualizar valor lógico
        self.cartesian_pos[axis] += delta
        
        # 2. Preparar comando
        x, y, z = self.cartesian_pos['x'], self.cartesian_pos['y'], self.cartesian_pos['z']
        p, r = self.cartesian_pos['pitch'], self.cartesian_pos['roll']
        speed = self.speed_var.get()
        mode = self.cartesian_mode.get() # MOVED o MOVEL

        # Validar alcance antes de enviar
        reachable, reason = self.is_reachable(x, y, z, p, r)
        if not reachable:
            messagebox.showwarning("Posición No Alcanzable", f"No se puede mover a esta posición: {reason}")
            # Revertir el cambio
            self.cartesian_pos[axis] -= delta
            # Actualizar GUI con el valor revertido
            self.cartesian_vars[axis].set(str(self.cartesian_pos[axis]))
            return

        # Comando ACL: <MODO> <X> <Y> <Z> <PITCH> <ROLL> <VEL>
        command = f"{mode} {x} {y} {z} {p} {r} SPEED={speed}\n"
        
        # 3. Actualizar GUI
        self.cartesian_vars[axis].set(str(self.cartesian_pos[axis]))

        # 4. Grabación (Guardamos el comando cartesiano si es necesario)
        if self.recording:
            # Nota: Para simplificar, las secuencias suelen grabarse como Joint positions
            # pero puedes adaptar tu playback para reconocer comandos tipo "CART"
            self.sequence.append(("CART", (mode, x, y, z, p, r), speed))
            self.update_sequence_display()

        if self.serial_conn:
            try:
                self.serial_conn.write(command.encode())
                self.log_message(f"SENT: {command.strip()}")
                print(f"Enviado: {command}")
            except Exception as e:
                self.log_message(f"ERROR: {e}")
                messagebox.showerror("Error", str(e))

    def start_recording(self):
        self.recording = True
        self.sequence.clear()
        messagebox.showinfo("Recording", "Started recording sequence")

    def stop_recording(self):
        self.recording = False
        messagebox.showinfo("Recording", "Stopped recording sequence")

    def clear_sequence(self):
        self.sequence.clear()
        self.update_sequence_display()

    def save_sequence(self):
        if not self.sequence:
            messagebox.showwarning("Warning", "No sequence to save")
            return
        file_path = filedialog.asksaveasfilename(defaultextension=".seq", filetypes=[("Sequence files", "*.seq"), ("All files", "*.*")])
        if file_path:
            try:
                with open(file_path, 'wb') as f:
                    pickle.dump(self.sequence, f)
                messagebox.showinfo("Success", "Sequence saved successfully")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save sequence: {e}")

    def load_sequence(self):
        file_path = filedialog.askopenfilename(filetypes=[("Sequence files", "*.seq"), ("All files", "*.*")])
        if file_path:
            try:
                with open(file_path, 'rb') as f:
                    self.sequence = pickle.load(f)
                self.update_sequence_display()
                messagebox.showinfo("Success", "Sequence loaded successfully")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load sequence: {e}")

    def play_sequence(self):
        if not self.sequence:
            messagebox.showwarning("Warning", "No sequence to play")
            return
        if not self.serial_conn:
            messagebox.showwarning("Warning", "Not connected to robot")
            return
        if not self.homed:
            messagebox.showwarning("Homing Required", "Please perform homing before playing sequences.")
            return
        self.playing = True
        threading.Thread(target=self._play_sequence).start()

    def _play_sequence(self):
        for item in self.sequence:
            if not self.playing:
                break
            if self.serial_conn:
                if len(item) == 3 and isinstance(item[0], str) and item[0] != "CART":
                    # Joint movement
                    joint, pos, speed = item
                    command = f"MOVEA {joint.upper()} {pos} SPEED={speed}\n"
                elif len(item) == 3 and item[0] == "CART":
                    # Cartesian movement
                    _, (mode, x, y, z, p, r), speed = item
                    command = f"{mode} {x} {y} {z} {p} {r} SPEED={speed}\n"
                else:
                    continue  # Skip unknown items
                self.serial_conn.write(command.encode())
                self.log_message(f"SENT: {command.strip()}")
                # Wait for confirmation from robot
                response = self.serial_conn.readline().decode().strip()
                self.log_message(f"RESP: {response}")
                if response == "":
                    messagebox.showerror("Error", f"Timeout waiting for response on {item}")
                    break
                elif "ERR" in response.upper():
                    messagebox.showerror("Error", f"Command failed: {response}")
                    break
                
                # Actualizar posiciones en la GUI
                if len(item) == 3 and isinstance(item[0], str) and item[0] != "CART":
                    # Joint movement
                    joint, pos, speed = item
                    self.joints[joint] = pos
                    self.joint_vars[joint].set(str(pos))
                elif len(item) == 3 and item[0] == "CART":
                    # Cartesian movement
                    _, (mode, x, y, z, p, r), speed = item
                    self.cartesian_pos['x'] = x
                    self.cartesian_pos['y'] = y
                    self.cartesian_pos['z'] = z
                    self.cartesian_pos['pitch'] = p
                    self.cartesian_pos['roll'] = r
                    self.cartesian_vars['x'].set(str(x))
                    self.cartesian_vars['y'].set(str(y))
                    self.cartesian_vars['z'].set(str(z))
                    self.cartesian_vars['pitch'].set(str(p))
                    self.cartesian_vars['roll'].set(str(r))
                
                # Add delay based on speed for Scorbot processing
                time.sleep(0.1 + (100 - speed) * 0.005)  # Adjust delay based on speed
        self.playing = False

    def stop_playback(self):
        self.playing = False

    def update_sequence_display(self):
        self.sequence_text.config(state=tk.NORMAL)
        self.sequence_text.delete(1.0, tk.END)
        for i, item in enumerate(self.sequence):
            if len(item) == 3 and isinstance(item[0], str) and item[0] != "CART":
                # Joint movement: (joint, pos, speed)
                joint, pos, speed = item
                self.sequence_text.insert(tk.END, f"{i+1}. {joint}: {pos} @ {speed}\n")
            elif len(item) == 3 and item[0] == "CART":
                # Cartesian movement: ("CART", (mode, x, y, z, p, r), speed)
                _, (mode, x, y, z, p, r), speed = item
                self.sequence_text.insert(tk.END, f"{i+1}. CART {mode}: X{x} Y{y} Z{z} P{p} R{r} @ {speed}\n")
            else:
                self.sequence_text.insert(tk.END, f"{i+1}. Unknown: {item}\n")
        self.sequence_text.config(state=tk.DISABLED)

    def on_closing(self):
        self.playing = False
        if self.serial_conn:
            self.serial_conn.close()
        self.root.destroy()

    def emergency_stop(self):
        self.playing = False
        if self.serial_conn:
            self.serial_conn.write(b"ABORT\n")  # Adjust to your firmware's stop command
            self.serial_conn.flush()
            self.log_message("SENT: ABORT")
        messagebox.showwarning("Emergency", "Stop command sent!")

    def calibrate_system(self):
        """Ejecuta una rutina automática de prueba para todos los ejes"""
        if not self.serial_conn:
            messagebox.showerror("Error", "Debe estar conectado al robot para calibrar.")
            return
        
        if not self.homed:
            messagebox.showwarning("Atención", "Se recomienda hacer HOME antes de la calibración.")
            
        if not messagebox.askyesno("Confirmar", "El robot se moverá a sus límites. ¿Área despejada?"):
            return

        # Definimos los puntos de prueba (Joint, Posición Objetivo)
        test_points = [
            ('base', 45), ('base', -45), ('base', 0),
            ('shoulder', 30), ('shoulder', -30), ('shoulder', 0),
            ('elbow', 30), ('elbow', -30), ('elbow', 0),
            ('wrist', 90), ('wrist', -90), ('wrist', 0),
            ('gripper', 100), ('gripper', 0)
        ]

        def run_calibration():
            self.playing = True
            try:
                for joint, pos in test_points:
                    if not self.playing: break
                    
                    print(f"Probando {joint} a {pos} unidades...")
                    
                    # Intentamos enviar el comando
                    command = f"MOVEA {joint.upper()} {pos} SPEED=30\n"
                    self.serial_conn.write(command.encode())
                    self.log_message(f"SENT: {command.strip()}")
                    
                    # Esperamos respuesta del controlador
                    response = self.serial_conn.readline().decode().strip()
                    self.log_message(f"RESP: {response}")
                    
                    if "ERR" in response.upper():
                        raise Exception(f"Error en {joint}: {response}")

                    # Actualizamos la GUI de forma segura desde el hilo
                    self.root.after(0, lambda j=joint, p=pos: self.update_gui_after_move(j, p))
                    
                    time.sleep(1.5) # Tiempo para que el movimiento físico ocurra
                    
                messagebox.showinfo("Calibración", "Rutina completada con éxito.")
            
            except Exception as e:
                messagebox.showerror("Fallo en Calibración", f"Se detuvo la prueba: {e}")
            finally:
                self.playing = False

        # Ejecutar en un hilo aparte para no congelar la ventana
        threading.Thread(target=run_calibration, daemon=True).start()

    def update_gui_after_move(self, joint, pos):
        """Actualiza los valores internos y la GUI después de un movimiento de calibración"""
        self.joints[joint] = pos
        self.joint_vars[joint].set(str(pos))

if __name__ == "__main__":
    root = tk.Tk()
    app = ScorbotController(root)
    root.mainloop()