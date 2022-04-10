import os
import multiprocessing as mp
import subprocess
import time

world_num = 1
batch_num = 24
worker_num = 4


class Worker(mp.Process):
    def __init__(self, world_index, batch_index, worker_index):
        super(Worker, self).__init__()
        self.name = 'worker_%i' % worker_index
        self.worker_index = worker_index
        self.world_index = world_index.value
        self.batch_index = batch_index.value+worker_index

    def run(self):
        os.environ['ROS_MASTER_URI'] = "http://localhost:1135" + \
            str(self.worker_index)
        os.environ['GAZEBO_MASTER_URI'] = "http://localhost:1134" + \
            str(self.worker_index)

        subprocess.call('python ' + os.path.dirname(__file__) +
                        '/run_batch_simulation_single.py ' + str(self.world_index) + ' ' + str(self.batch_index), shell=True)


if __name__ == "__main__":

    for i in range(world_num):
        for j in range(batch_num/worker_num):
            # initializing some shared values between process
            world_index = mp.Value('i', 0)
            world_index.value = i
            batch_index = mp.Value('i', 0)
            batch_index.value = j*worker_num

            # parallel training
            workers = [Worker(world_index, batch_index, k)
                       for k in range(worker_num)]
            # [w.start() for w in workers]
            for w in workers:
                w.start()
                time.sleep(1)
            [w.join() for w in workers]
