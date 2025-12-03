# garra_final_v4_antibug.py
import pybullet as p
import pybullet_data
import time
import numpy as np
import requests
import math
import random
import traceback

# --- DEFINIÇÃO DA POSTURA CONFORTÁVEL (HOME) ---
HOME_POSE = [0, -0.6, 0, -1.75, 0, 1.0, 0]

# --- CONFIGURAÇÕES GLOBAIS ---
ROBOT_BASE_X = -0.1
ROBOT_BASE_Y = 0.0

NODE_RED_URL = "http://localhost:1880/dados-robo-garra"
# --- SUBTITUIR A FUNÇÃO send_to_node_red EXISTENTE POR ESTA ---
def send_to_node_red(data_dict, timeout=0.05):
    try:
        # Timeout curto para não travar a simulação
        response = requests.post(NODE_RED_URL, json=data_dict, timeout=timeout)
        if response.status_code == 200:
            return response.json() # Retorna o comando (ex: {"comando": "ligar"})
    except Exception:
        pass
    return None

TABLE_HEIGHT = 0.625
BOX_X = 0.5
BOX_HEIGHT = 0.12
H_SAFE = TABLE_HEIGHT + BOX_HEIGHT + 0.3

debug_items = []
def draw_state(robot_pos, target_pos, state_name, extra_info=""):
    global debug_items
    for item in debug_items:
        try:
            p.removeUserDebugItem(item)
        except Exception:
            pass
    debug_items.clear()
    if target_pos is not None:
        try:
            debug_items.append(p.addUserDebugLine(robot_pos, target_pos, [0, 1, 0], lineWidth=4))
        except Exception:
            pass
    msg = f"[{state_name}]\n{extra_info}"
    try:
        debug_items.append(p.addUserDebugText(msg, [robot_pos[0], robot_pos[1], robot_pos[2]+0.35], [0,0,0], textSize=1.2))
    except Exception:
        pass

def create_environment():
    p.loadURDF("plane.urdf")
    p.loadURDF("table/table.urdf", basePosition=[0.5, 0, 0.0], useFixedBase=True)

    def create_box(pos, color):
        thickness = 0.01
        size = 0.12
        height = BOX_HEIGHT
        base_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, thickness], rgbaColor=color)
        base_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, thickness])
        wall_h_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[thickness, size, height/2])
        wall_v_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, thickness, height/2])

        base_z = pos[2] + thickness
        p.createMultiBody(0, base_col, base_vis, basePosition=[pos[0], pos[1], base_z])
        z_wall = base_z + height/2

        wall_vis_h = p.createVisualShape(p.GEOM_BOX, halfExtents=[thickness, size, height/2], rgbaColor=color)
        wall_vis_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, thickness, height/2], rgbaColor=color)

        p.createMultiBody(0, wall_h_c, wall_vis_h, basePosition=[pos[0] + size, pos[1], z_wall])
        p.createMultiBody(0, wall_h_c, wall_vis_h, basePosition=[pos[0] - size, pos[1], z_wall])
        p.createMultiBody(0, wall_v_c, wall_vis_v, basePosition=[pos[0], pos[1] + size, z_wall])
        p.createMultiBody(0, wall_v_c, wall_vis_v, basePosition=[pos[0], pos[1] - size, z_wall])

    create_box([BOX_X, 0, TABLE_HEIGHT], [1, 0, 0, 1])
    create_box([BOX_X, 0.28, TABLE_HEIGHT], [0, 1, 0, 1])
    create_box([BOX_X, -0.28, TABLE_HEIGHT], [0, 0, 1, 1])

def spawn_objects():
    objects = []
    spawn_z = TABLE_HEIGHT + 0.08
    
    cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.015, 0.015, 0.015])
    cube_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.015, 0.015, 0.015], rgbaColor=[0, 1, 0, 1])
    sphere_col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.015)
    blue_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.015, rgbaColor=[0, 0, 1, 1])
    red_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.015, rgbaColor=[1, 0, 0, 1])

    spawn_list = []
    for _ in range(10): spawn_list.append(('cube_green', cube_col, cube_vis, 0.1))
    for _ in range(10): spawn_list.append(('sphere_blue', sphere_col, blue_vis, 0.5))
    for _ in range(5): spawn_list.append(('sphere_red', sphere_col, red_vis, 2.0))
    random.shuffle(spawn_list)

    for i, (obj_type, col, vis, mass_val) in enumerate(spawn_list):
        x = BOX_X + random.uniform(-0.08, 0.08)
        y = random.uniform(-0.08, 0.08)
        z = spawn_z + (i * 0.02)
        uid = p.createMultiBody(baseMass=mass_val, baseCollisionShapeIndex=col,
                                baseVisualShapeIndex=vis, basePosition=[x, y, z])
        p.changeDynamics(uid, -1, rollingFriction=0.01, spinningFriction=0.01, lateralFriction=1.0)
        objects.append({'id': uid, 'type': obj_type})
    return objects

# --- FUNÇÃO DE MOVIMENTO REALISTA (NULL SPACE CORRIGIDO) ---
def move_arm_simple(robot_id, target_pos, target_orn, end_effector_index, max_force=800.0):
    num_joints = p.getNumJoints(robot_id)

    ll = [-2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -3.05]
    ul = [2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 3.05]
    jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]

    if len(ll) < num_joints:
        diff = num_joints - len(ll)
        ll += [-3.0] * diff
        ul += [3.0] * diff
        jr += [6.0] * diff

    dx = target_pos[0] - ROBOT_BASE_X
    dy = target_pos[1] - ROBOT_BASE_Y
    base_angle = math.atan2(dy, dx)

    # --- CORREÇÃO AQUI ---
    # Rest Pose mais agressivo para manter o cotovelo para cima
    # [BaseDinâmica, OmbroLevementeFrente, 0, CotoveloBemDobrado, 0, PulsoCompensa, 0]
    rp = [base_angle, -0.4, 0, -1.8, 0, 1.0, 0] + [0]*(num_joints-7)
    # ---------------------

    if target_orn is None:
        target_orn = p.getQuaternionFromEuler([0, math.pi, 0])

    try:
        joint_poses = p.calculateInverseKinematics(
            robot_id,
            end_effector_index,
            target_pos,
            targetOrientation=target_orn,
            lowerLimits=ll,
            upperLimits=ul,
            jointRanges=jr,
            restPoses=rp,
            jointDamping=[0.05] * num_joints,
            maxNumIterations=100
        )

        idx_movel = 0
        for j in range(num_joints):
            info = p.getJointInfo(robot_id, j)
            q_index = info[3] 
            if q_index > -1:
                if idx_movel < len(joint_poses):
                    p.setJointMotorControl2(
                        robot_id, j, p.POSITION_CONTROL,
                        targetPosition=joint_poses[idx_movel],
                        force=max_force
                    )
                    idx_movel += 1
    except Exception as e:
        print(f"Erro IK: {e}")

def check_reached(current_pos, target_pos, tolerance=0.035):
    try:
        c = np.array(current_pos)
        t = np.array(target_pos)
        return np.linalg.norm(c - t) < tolerance
    except Exception:
        return False
# --- INSERIR ANTES DE def main(): ---
def calculate_metrics(robot_id, current_ee_pos, target_pos_ref):
    # 1. Erro de Posição (Distância Euclidiana entre onde está e onde deveria estar)
    if target_pos_ref is not None:
        # Converte para array numpy para facilitar o cálculo
        pos_error = np.linalg.norm(np.array(current_ee_pos) - np.array(target_pos_ref))
    else:
        pos_error = 0.0
    
    # 2. Energia Estimada (Soma absoluta dos torques aplicados em todas as juntas)
    total_energy = 0.0
    num_joints = p.getNumJoints(robot_id)
    for j in range(num_joints):
        # getJointState retorna [pos, vel, reaction, applied_torque]
        # O torque é o índice 3
        try:
            state = p.getJointState(robot_id, j)
            torque = state[3]
            total_energy += abs(torque)
        except:
            pass
            
    return pos_error, total_energy

def main():
    try:
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=90, cameraPitch=-30, cameraTargetPosition=[0.5, 0, 0.6])

        create_environment()
        robot_start = [ROBOT_BASE_X, ROBOT_BASE_Y, TABLE_HEIGHT]
        
        robot_id = p.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")[0]
        p.resetBasePositionAndOrientation(robot_id, robot_start, [0,0,0,1])
        
        end_effector_index = None
        for j in reversed(range(p.getNumJoints(robot_id))):
            info = p.getJointInfo(robot_id, j)
            name = info[1].decode() if isinstance(info[1], bytes) else info[1]
            if "tip" in name.lower() or "finger" in name.lower():
                end_effector_index = j
                break
        if end_effector_index is None:
            end_effector_index = 10 
        print("End effector index usado:", end_effector_index)

        # Reset inicial usando HOME_POSE
        idx_movel = 0
        for j in range(p.getNumJoints(robot_id)):
            if p.getJointInfo(robot_id, j)[2] != p.JOINT_FIXED:
                if idx_movel < len(HOME_POSE):
                    try:
                        p.resetJointState(robot_id, j, HOME_POSE[idx_movel])
                    except Exception:
                        pass
                idx_movel += 1

        objects = spawn_objects()

        for _ in range(200):
            p.stepSimulation()
            time.sleep(1./240.)

        gripper_orn = p.getQuaternionFromEuler([0, math.pi, 0])
        sorted_count = {"cube_green": 0, "sphere_blue": 0}
        current_target = None
        current_weight = 0.0

        state = "INIT"
        grasp_constraint = None
        state_timer = time.time()
        grab_wait_counter = 0

        step = 0
        target_sequence = []
        last_send_time = 0.0

        print("--- ROBÔ FINAL (V4 - ANTI-BUG) ---")

        # --- ADICIONE ESTAS VARIÁVEIS AQUI ---
        system_active = False  # Começa PARADO aguardando o Node-RED
        current_target_pos_ref = [0.2, 0, H_SAFE] # Referência inicial para cálculo de erro
        frame_count = 0 # Para controlar frequência de envio
        # -------------------------------------

        while p.isConnected():
            # --- INSERIR LOGO NO INÍCIO DO WHILE ---
            frame_count += 1
            ee_pos = p.getLinkState(robot_id, end_effector_index)[0]
        
            # A. Define a referência para cálculo do erro baseado no estado atual
            # Isso garante que o gráfico de erro faça sentido
            if state == "INIT" or state == "SEARCH" or state == "FINISHED":
                current_target_pos_ref = [0.2, 0, H_SAFE]
            elif (state == "MOVE_L_SHAPE" or state == "LIFT" or state == "TRANSPORT") and len(target_sequence) > 0:
                # Se está movendo, o erro é em relação ao alvo atual da sequência
                safe_step = min(step, len(target_sequence)-1)
                current_target_pos_ref = target_sequence[safe_step]
            elif state == "GRAB":
                # No grab, assumimos que ele está onde deveria estar (erro tende a zero)
                current_target_pos_ref = ee_pos 

            # B. Calcula as Métricas
            val_erro, val_energia = calculate_metrics(robot_id, ee_pos, current_target_pos_ref)

            # C. Comunicação Bidirecional (Envia e Recebe a cada 10 frames para não pesar)
            if frame_count % 10 == 0:
                payload = {
                    "system": "Arm_1A",
                    "active": system_active, # Informa ao dashboard se está rodando
                    "state": state,
                    "metrics": {
                        "error": float(val_erro),
                        "energy": float(val_energia),
                        "weight": float(current_weight)
                    },
                    "counts": sorted_count
                }
                # Envia e captura a resposta
                resposta = send_to_node_red(payload)

                # --- ADICIONE ESTA LINHA ---
                print(f"RESPOSTA DO SERVIDOR: {resposta}") 
                # ---------------------------
            
                # Processa o comando recebido do botão
                if resposta and "comando" in resposta:
                    if resposta["comando"] == "ligar":
                        system_active = True
                    elif resposta["comando"] == "desligar":
                        system_active = False

            # D. Freio de Mão (Se estiver desligado, não executa a máquina de estados)
            if not system_active:
                draw_state(ee_pos, None, "PAUSADO", "Aguardando Start no Node-RED...")
                p.stepSimulation()
                time.sleep(1./240.)
                continue # Pula todo o resto do loop e volta pro início
        
        # ----------------------------------------
        # DAQUI PARA BAIXO SEGUE SEU CÓDIGO ORIGINAL DA MÁQUINA DE ESTADOS
            t = time.time()
            ee_pos = p.getLinkState(robot_id, end_effector_index)[0]

            if state not in ["SEARCH", "FINISHED", "WAIT_DROP"] and (t - state_timer > 15.0):
                print(f"[TIMEOUT] Resetando... estado={state}")
                if grasp_constraint:
                    try: p.removeConstraint(grasp_constraint)
                    except: pass
                    grasp_constraint = None
                state = "INIT"
                state_timer = t

            if state == "INIT":
                target = [0.2, 0, H_SAFE]
                move_arm_simple(robot_id, target, gripper_orn, end_effector_index)
                draw_state(ee_pos, target, "INICIALIZANDO")
                if check_reached(ee_pos, target):
                    state = "SEARCH"
                    state_timer = t

            elif state == "SEARCH":
                candidates = []
                alive_objects = []
                for obj in objects:
                    try:
                        pos, _ = p.getBasePositionAndOrientation(obj['id'])
                        if (BOX_X - 0.09) < pos[0] < (BOX_X + 0.09) and -0.09 < pos[1] < 0.09 and pos[2] > (TABLE_HEIGHT + 0.02):
                            if obj['type'] != 'sphere_red':
                                candidates.append(obj)
                        alive_objects.append(obj)
                    except Exception:
                        pass
                objects = alive_objects

                if not candidates:
                    state = "FINISHED"
                else:
                    candidates.sort(key=lambda o: p.getBasePositionAndOrientation(o['id'])[0][2], reverse=True)
                    current_target = candidates[0]
                    print(f"Alvo selecionado: {current_target['type']} (id={current_target['id']})")
                    t_pos, _ = p.getBasePositionAndOrientation(current_target['id'])
                    
                    target_sequence = [
                        [ee_pos[0], ee_pos[1], H_SAFE],
                        [t_pos[0], t_pos[1], H_SAFE],
                        [t_pos[0], t_pos[1], t_pos[2] + 0.04]
                    ]
                    step = 0
                    state = "MOVE_L_SHAPE"
                    state_timer = t

            elif state == "MOVE_L_SHAPE":
                if step == 2 and current_target:
                    try:
                        cur_pos, _ = p.getBasePositionAndOrientation(current_target['id'])
                        target_sequence[2] = [cur_pos[0], cur_pos[1], target_sequence[2][2]]
                    except Exception:
                        pass

                if step < len(target_sequence):
                    target = target_sequence[step]
                    move_arm_simple(robot_id, target, gripper_orn, end_effector_index)
                    step_names = ["SUBINDO", "MOVENDO LATERAL", "DESCENDO"]
                    draw_state(ee_pos, target, step_names[step])

                    if check_reached(ee_pos, target):
                        step += 1
                        if step >= len(target_sequence):
                            grab_wait_counter = 0
                            state = "GRAB"
                            state_timer = t

            elif state == "GRAB":
                if current_target is None:
                    state = "SEARCH"
                    continue

                if grasp_constraint is None:
                    try:
                        grasp_constraint = p.createConstraint(
                            robot_id, end_effector_index, current_target['id'], -1,
                            p.JOINT_POINT2POINT, [0, 0, 0], [0, 0, 0], [0, 0, 0]
                        )
                        p.changeConstraint(grasp_constraint, maxForce=2000)
                        print("[MAGNETISMO LIGADO]")
                    except Exception as e:
                        print("[GRAB] falha constraint:", e)
                        grasp_constraint = None
                        state = "SEARCH"
                        continue

                grab_wait_counter += 1
                draw_state(ee_pos, None, "PEGANDO", f"{grab_wait_counter}/80")
                
                current_pos = p.getLinkState(robot_id, end_effector_index)[0]
                move_arm_simple(robot_id, current_pos, gripper_orn, end_effector_index)

                if grab_wait_counter > 80:
                    try: current_weight = p.getDynamicsInfo(current_target['id'], -1)[0]
                    except: current_weight = 0.0
                    print(f"Peso: {current_weight} kg")

                    target_sequence = [
                        [ee_pos[0], ee_pos[1], ee_pos[2] + 0.1],
                        [ee_pos[0], ee_pos[1], H_SAFE]
                    ]
                    step = 0
                    state = "LIFT"
                    state_timer = t

            elif state == "LIFT":
                if step < len(target_sequence):
                    target = target_sequence[step]
                    move_arm_simple(robot_id, target, gripper_orn, end_effector_index)
                    draw_state(ee_pos, target, "LEVANTANDO")

                    try:
                        obj_pos, _ = p.getBasePositionAndOrientation(current_target['id'])
                        if obj_pos[2] < (TABLE_HEIGHT + 0.05) and ee_pos[2] > (TABLE_HEIGHT + 0.2):
                            print("Objeto caiu!")
                            if grasp_constraint:
                                try: p.removeConstraint(grasp_constraint)
                                except: pass
                                grasp_constraint = None
                            state = "SEARCH"
                            continue
                    except: pass

                    if check_reached(ee_pos, target):
                        step += 1
                        if step >= len(target_sequence):
                            state = "TRANSPORT"
                            state_timer = t

            elif state == "TRANSPORT":
                if current_target is None:
                    state = "SEARCH"
                    continue
                
                if current_target['type'] == 'cube_green':
                    dest = [BOX_X, 0.28, H_SAFE]
                else:
                    dest = [BOX_X, -0.28, H_SAFE]

                move_arm_simple(robot_id, dest, gripper_orn, end_effector_index)
                draw_state(ee_pos, dest, "TRANSPORTANDO")

                if check_reached(ee_pos, dest):
                    state = "DROP"
                    state_timer = t

            elif state == "DROP":
                drop_pos = [ee_pos[0], ee_pos[1], H_SAFE - 0.1]
                move_arm_simple(robot_id, drop_pos, gripper_orn, end_effector_index)
                draw_state(ee_pos, drop_pos, "SOLTANDO")

                if check_reached(ee_pos, drop_pos):
                    if grasp_constraint:
                        try: p.removeConstraint(grasp_constraint)
                        except: pass
                        grasp_constraint = None
                        print("[MAGNETISMO DESLIGADO]")

                    if current_target and current_target['type'] in sorted_count:
                        sorted_count[current_target['type']] += 1
                    print(f"Sucesso! Placar: {sorted_count}")
                    
                    state = "WAIT_DROP"
                    state_timer = t

            elif state == "WAIT_DROP":
                hold_pos = [ee_pos[0], ee_pos[1], H_SAFE - 0.1]
                move_arm_simple(robot_id, hold_pos, gripper_orn, end_effector_index)
                
                remain = 2.0 - (t - state_timer)
                draw_state(ee_pos, hold_pos, "AGUARDANDO...", f"{remain:.1f}s")
                
                if (t - state_timer) > 2.0:
                    state = "RETURN"
                    state_timer = t

            elif state == "RETURN":
                home = [0.2, 0, H_SAFE]
                move_arm_simple(robot_id, home, gripper_orn, end_effector_index)
                draw_state(ee_pos, home, "VOLTANDO")
                if check_reached(ee_pos, home):
                    state = "SEARCH"
                    state_timer = t

            elif state == "FINISHED":
                final_pos = [0, 0, H_SAFE]
                move_arm_simple(robot_id, final_pos, gripper_orn, end_effector_index)
                draw_state(ee_pos, final_pos, "CONCLUÍDO", f"Total: {sum(sorted_count.values())}")

            p.stepSimulation()
            time.sleep(1./240.)

            #if (t - last_send_time) > 0.5:
             #   payload = {
              #      "system": "Robô Final NullSpace",
               #     "state": state,
                #    "weight_kg": current_weight,
                 #   "counts": sorted_count,
                  #  "position": list(ee_pos)
               # }
               # send_to_node_red(payload)
               # last_send_time = t

    except KeyboardInterrupt:
        print("Interrompido.")
    except Exception as e:
        print("Erro crítico:", e)
        traceback.print_exc()
    finally:
        try: p.disconnect()
        except: pass

if __name__ == "__main__":
    main()