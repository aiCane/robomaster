from Thu_Aug_29.task_1 import task_1_main
from Thu_Aug_29.task_2 import task_2_main
from Thu_Aug_29.task_3 import task_3_main

from Thu_Aug_29.__init__ import ep_robot

if __name__ == '__main__':
    task_1_main()
    task_2_main()
    task_3_main(3)

    ep_robot.close()
