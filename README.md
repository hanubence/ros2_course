### Szorgalmi feladat 2024. 04. 12-ig
- Implementáljunk metódust, amely r sugrú körív mentén mozgatja a megfogót.

  - A psm_grasp controllerben létrehoztam egy új circle_around_marker(self, v, omega, dt, r) [függvényt](https://github.com/hanubence/ros2_course/blob/main/src/ros2_course/ros2_course/psm_grasp.py)
    - A körpályát az órán használt dummy marker körül tesszük meg
  - A paraméterekkel befolyásolhatjuk a leírt körpálya méretet, sebességet, stb.
#### Példák:
  - Kis körív (r) esetén, nagyobb sebességgel:

    ![small-radius](https://github.com/hanubence/ros2_course/assets/32911312/63c885a7-f15b-4603-be26-2fb3cafda2f3)
    
  - Nagy körív (r) esetén, alacsonyabb sebességgel:

    ![large-radius](https://github.com/hanubence/ros2_course/assets/32911312/f11b406c-c626-49d9-be52-8529f5935372)
  
