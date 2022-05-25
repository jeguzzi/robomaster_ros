- prob. 1024 units = 180 degrees
=> 0.17578125 deg/unit

at least formally


The questions are:

- do all robots have the same range (or at least angles when fully contracted)?

- do they correspond to the urdf model?

- are the servo calibrated, i.e. is the ratio unit / angle constant?

  - servo1:
    - low: 180.0, 1024
    - high: 131.1, 746
    - delta: 48.9
  - servo2:
    - flexed: 183.6, 1045  (174.9, 177.8, 178.2, 181.7, 179.1)
    - extended: 89.4, 509 (85.9, 88.0, 88.7, 86.4)
    - delta: 94.2

    perché tutte queste variazioni? sembrano maggiori per servo 2

  - measured with iphone:
    - servo1:
      - low: 31 deg (arm)
      - high: -18 deg
      - delta: 49 deg
    - servo2:
      - flexed: 75 deg (arm)
      - extended: 171 deg
      - delta: 96 deg

  - urdf/coppelia:
    - servo1:
      - low: 31.0 deg
      - high: -17.8 deg
      - delta: 48.8 deg    (joint [-3.0, 45.8] = 48.8) (actually [-4.2 ...])
      - 0 (not running): 28.5 deg (arm)
    - servo2:
      - flexed: 90-13.7 = 76.3 deg
      - extended: 90 + 81.3 = 171.3 deg
      - delta: 95 deg (join [-15.7, 79.3] = 95)
      - 0 (not running): 92.0 deg (arm)

    servo1 - servo0 in [-19.9 deg, 85.9 deg]

  - in sim:
    - servo 1:
      - 0 = 27.7 deg (arm) = 2.6578f
      - low = 1.7313f => (a - zero) =
      - high = -0.7993f


    - servo 2:
      - 0 = (180 - 88.9) deg (arm)
      => low = 88.9 - 15.7 = ??? 75.4


      motors.right.min_angle = std::max(motors.left.angle.current - 0.3473f, -0.2740f);
      motors.right.max_angle = std::min(motors.left.angle.current + 1.2147f, 1.3840f);
      motors.left.min_angle = std::max(motors.right.angle.current - 1.2147f, -0.7993f);
      motors.left.max_angle = motors.right.angle.current + 0.3473f;

      => flexed angle = -0.2740 + 0.3473 = 0.0733 = 4.2 deg

      coherent with the urdf model
      urdf:

      servo2 - servo 1 in [-0.34732, 1.21475]
      servo1 in [-0.79936, 1.73137]
      servo2 in [-0.274, 1.384]


arm size:

- coppelia / urd
bottom 0.12104
top: 0.11980
