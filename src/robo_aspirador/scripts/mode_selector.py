#!/usr/bin/env python3
import os
import sys
import time

import rospy
import rospkg
import roslaunch
import tf
import tf2_ros


def _open_tty():
    try:
        return open("/dev/tty", "r")
    except Exception:
        return sys.stdin


def _prompt(tty, text):
    sys.stdout.write(text)
    sys.stdout.flush()
    line = tty.readline()
    if not line:
        return ""
    return line.strip()


class ModeSelector:
    def __init__(self, base_launch, manual_launch, auto_launch, tf_buffer, frames, use_current_pose):
        self.base_launch = base_launch
        self.manual_launch = manual_launch
        self.auto_launch = auto_launch
        self.tf_buffer = tf_buffer
        self.frames = frames
        self.use_current_pose = use_current_pose
        self.base_parent = None
        self.mode_parent = None
        self.current_mode = None

    def _start_launch(self, launch_path, args):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        parent = roslaunch.parent.ROSLaunchParent(uuid, [(launch_path, args)])
        parent.start()
        return parent

    def start_base(self):
        if self.base_parent is None:
            self.base_parent = self._start_launch(self.base_launch, [])
            sys.stdout.write("Base do robô iniciada.\n")
            sys.stdout.flush()

    def stop_mode(self):
        if self.mode_parent is not None:
            self.mode_parent.shutdown()
            self.mode_parent = None
            self.current_mode = None
            time.sleep(0.5)

    def start_manual(self):
        if self.current_mode == "manual":
            sys.stdout.write("Modo manual já está ativo.\n")
            sys.stdout.flush()
            return
        self.stop_mode()
        self.mode_parent = self._start_launch(self.manual_launch, [])
        self.current_mode = "manual"
        sys.stdout.write("Modo MANUAL de mapeamento iniciado.\n")
        sys.stdout.flush()

    def _get_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frames["map"],
                self.frames["base"],
                rospy.Time(0),
                rospy.Duration(0.5),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = tf.transformations.euler_from_quaternion(
                [rotation.x, rotation.y, rotation.z, rotation.w]
            )[2]
            return {"x": translation.x, "y": translation.y, "a": yaw}
        except Exception:
            return None

    def start_auto(self, map_path, rviz_enabled, initial_pose, radius_params):
        if self.current_mode == "auto":
            sys.stdout.write("Modo automático já está ativo.\n")
            sys.stdout.flush()
            return
        if self.use_current_pose:
            current_pose = self._get_current_pose()
            if current_pose is not None:
                initial_pose = current_pose
        self.stop_mode()
        args = [
            "map:={}".format(map_path),
            "rviz:={}".format(str(rviz_enabled).lower()),
            "initial_pose_x:={}".format(initial_pose["x"]),
            "initial_pose_y:={}".format(initial_pose["y"]),
            "initial_pose_a:={}".format(initial_pose["a"]),
            "robot_radius:={}".format(radius_params["robot"]),
            "tool_radius:={}".format(radius_params["tool"]),
        ]
        self.mode_parent = self._start_launch(self.auto_launch, args)
        self.current_mode = "auto"
        sys.stdout.write("Modo AUTOMÁTICO de aspiração iniciado.\n")
        sys.stdout.flush()

    def shutdown(self):
        self.stop_mode()
        if self.base_parent is not None:
            self.base_parent.shutdown()
            self.base_parent = None


def main():
    rospy.init_node("robo_aspirador_mode_selector", anonymous=False, disable_signals=True)
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("robo_aspirador")

    base_launch = os.path.join(pkg_path, "launch", "robo_aspirador_base.launch")
    manual_launch = os.path.join(pkg_path, "launch", "manual_mapping.launch")
    auto_launch = os.path.join(pkg_path, "launch", "automatic_aspiracao.launch")

    default_map = rospy.get_param("~default_map", "")
    if not default_map:
        default_map = os.path.join(rospack.get_path("tracking_pid"), "maps", "grid.yaml")
    rviz_enabled = rospy.get_param("~rviz", True)
    initial_pose = {
        "x": float(rospy.get_param("~initial_pose_x", 0.0)),
        "y": float(rospy.get_param("~initial_pose_y", 0.0)),
        "a": float(rospy.get_param("~initial_pose_a", 0.0)),
    }
    radius_params = {
        "robot": float(rospy.get_param("~robot_radius", 0.3)),
        "tool": float(rospy.get_param("~tool_radius", 0.3)),
    }

    use_current_pose = bool(rospy.get_param("~use_current_pose", True))
    frames = {
        "map": rospy.get_param("~map_frame", "map"),
        "base": rospy.get_param("~base_frame", "base_link"),
    }
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    selector = ModeSelector(base_launch, manual_launch, auto_launch, tf_buffer, frames, use_current_pose)
    selector.start_base()

    tty = _open_tty()
    try:
        menu = (
            "\nEscolha o modo:\n"
            "  1 - Manual (mapeamento + keyteleop)\n"
            "  2 - Automático (aspiração)\n"
            "  q - Sair\n"
        )
        sys.stdout.write(menu)
        sys.stdout.flush()
        while not rospy.is_shutdown():
            choice = _prompt(tty, "modo [1/2/q]> ").lower()

            if choice in ("1", "manual", "m"):
                selector.start_manual()
            elif choice in ("2", "auto", "a"):
                map_path = _prompt(
                    tty,
                    "Mapa .yaml para aspiração (ENTER para padrão): ",
                )
                if not map_path:
                    map_path = default_map
                selector.start_auto(map_path, rviz_enabled, initial_pose, radius_params)
            elif choice in ("q", "quit", "sair", "exit"):
                break
            elif choice == "":
                time.sleep(0.2)
            else:
                sys.stdout.write("Opção inválida.\n")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        selector.shutdown()


if __name__ == "__main__":
    main()
