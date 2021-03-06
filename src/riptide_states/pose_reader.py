import yaml


class PoseReader(object):

    def __init__(self, path):
        self.path = path

    def _read_file(self):
        with open(self.path, "r") as fp:
            return yaml.load(fp, Loader=yaml.FullLoader)

    @property
    def tasks(self):
        pose_file = self._read_file()
        return list(pose_file.keys())

    def get_task(self, name):
        return Task(self, name)


class Task(object):

    def __init__(self, reader, task_name):
        self.reader = reader
        self.task_name = task_name

    @property
    def poses(self):
        task = self.reader._read_file()[self.task_name]
        return list(task.keys())

    def get_pose(self, name):
        task = self.reader._read_file()[self.task_name]

        # TODO: Change the representation of this info
        return task[name]



if __name__ == "__main__":
    pose_reader = PoseReader("/home/blaine/osu-uwrt/riptide_software/src/riptide_states/cfg/task_poses.yaml")
    print(pose_reader.tasks)
    cutie_task = pose_reader.get_task("cutie")
    print(cutie_task.poses)
    for pose_name in cutie_task.poses:
        print(cutie_task.get_pose(pose_name))