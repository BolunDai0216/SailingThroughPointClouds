import hppfcl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle


class HeightMapSceneGenerator:
    def __init__(
        self,
        min_distance_between_objs=0.25,
        box_width_horizontal=0.4,
        box_height_horizontal=0.2,
        circle_radius_horizontal=0.1,
        box_width_vertical=0.2,
        box_height_vertical=0.3,
        circle_radius_vertical=0.1,
        n_horizontal_trials=50,
        n_vertical_trials=50,
    ):
        self.collision_manager = hppfcl.DynamicAABBTreeCollisionManager()
        self.callback = hppfcl.DistanceCallBackDefault()
        self.min_distance_between_objs = min_distance_between_objs

        self.box_width_horizontal = box_width_horizontal
        self.box_height_horizontal = box_height_horizontal
        self.circle_radius_horizontal = circle_radius_horizontal
        self.box_width_vertical = box_width_vertical
        self.box_height_vertical = box_height_vertical
        self.circle_radius_vertical = circle_radius_vertical

        self.n_horizontal_trials = n_horizontal_trials
        self.n_vertical_trials = n_vertical_trials

    def plt_reset(self):
        self.collision_manager = hppfcl.DynamicAABBTreeCollisionManager()
        self.fig, self.ax = plt.subplots(1, 1, figsize=(6, 6))
        self.ax.set_aspect("equal")
        self.ax.set_xlim(0, 2)
        self.ax.set_ylim(0, 2)
        self.fig.patch.set_facecolor("black")
        self.ax.set_facecolor("black")
        self.ax.axis("off")

    def plt_save(self, filename, save_dir="./height_map_img", dpi=50):
        plt.savefig(f"{save_dir}/{filename}.png", dpi=dpi)
        plt.close()

    def sample_obstacle(self, n_vertical, n_horizontal):
        for _ in range(n_vertical):
            self.sample_obstacles(
                self.n_vertical_trials,
                0.25,
                1.75,
                0.15,
                1.85,
                self.box_width_vertical,
                self.box_height_vertical,
                self.circle_radius_vertical,
            )

        for _ in range(n_horizontal):
            self.sample_obstacles(
                self.n_horizontal_trials,
                0.15,
                1.85,
                0.25,
                1.75,
                self.box_width_horizontal,
                self.box_height_horizontal,
                self.circle_radius_horizontal,
            )

    def sample_obstacles(
        self,
        n_trials,
        x_min,
        x_max,
        y_min,
        y_max,
        box_width,
        box_height,
        circle_radius,
    ):
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)

        for _ in range(n_trials):
            success = self.add_obj(
                x,
                y,
                box_width,
                box_height,
                circle_radius,
            )

            if success:
                break

        return success

    def add_obj(self, x, y, box_width, box_height, circle_radius, obj_z_height=0.6):
        obj_type = np.random.choice(["box", "circle"])

        if obj_type == "box":
            obj = hppfcl.CollisionObject(
                hppfcl.Box(box_width, box_height, obj_z_height)
            )
        elif obj_type == "circle":
            obj = hppfcl.CollisionObject(hppfcl.Cylinder(circle_radius, obj_z_height))

        T_obj = hppfcl.Transform3f(np.eye(3), np.array([[x], [y], [obj_z_height / 2]]))
        obj.setTransform(T_obj)
        success = self.add_obj_to_manager(obj, obj_type)

        if success:
            if obj_type == "box":
                self.add_plt_box(x, y, width=box_width, height=box_height)
            elif obj_type == "circle":
                self.add_plt_circle(x, y, radius=circle_radius)

        return success

    def add_obj_to_manager(self, obj, type):
        self.callback.data.result.clear()
        self.callback.data.done = False
        self.collision_manager.distance(obj, self.callback)
        distance = self.callback.data.result.min_distance
        success = False

        if distance >= self.min_distance_between_objs:
            self.collision_manager.registerObject(obj)
            if self.collision_manager.size() == 0:
                self.collision_manager.setup()
            else:
                self.collision_manager.update()

            x = obj.getTranslation()[0]
            y = obj.getTranslation()[1]

            if type == "box":
                width = obj.collisionGeometry().halfSide[0] * 2
                height = obj.collisionGeometry().halfSide[1] * 2
                self.add_plt_box(x, y, width, height)
            elif type == "circle":
                radius = obj.collisionGeometry().radius
                self.add_plt_circle(x, y, radius)
            else:
                raise ValueError("Invalid type, only 'box' and 'circle' are supported.")
            success = True

        return success

    def add_plt_box(self, x, y, width=0.1, height=0.1):
        lower_left_corner_x = x - width / 2
        lower_left_corner_y = y - height / 2

        self.ax.add_patch(
            Rectangle(
                (lower_left_corner_x, lower_left_corner_y),
                width,
                height,
                edgecolor="white",
                facecolor="white",
            )
        )

    def add_plt_circle(self, x, y, radius=0.1):
        self.ax.add_patch(Circle((x, y), radius, edgecolor="white", facecolor="white"))
