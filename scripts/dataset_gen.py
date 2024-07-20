from sailing_through_pcds.height_map_gen import HeightMapSceneGenerator


def main():
    gen = HeightMapSceneGenerator()

    for i in range(50):
        gen.plt_reset()
        gen.sample_obstacle(2, 3)
        filename = f"map{i:03}"
        gen.plt_save(
            filename, save_dir="/workspace/sailing-through-pcds-devcontainer/data"
        )


if __name__ == "__main__":
    main()
