# boxjs

Boxjs is a javascript 2D physics engine based off of Box2D.

Boxjs can be used both in node as well as in browser.

For a demo (WIP), check out [the testbed](https://ethanrutherford.github.io/boxjs/testing).

### testbed info/controls
- There are 9 tests, accessible with the number keys `1-9`.
- Click and drag anywhere to spawn a box with velocity.
- Shift + click to grab a body and drag it around.
- Press `0` to toggle debug view, which draws the broadphase AABBs.

#### test 1 - basic test
This test displays basic features of the physics engine. There are examples of various
shapes and joints, including a rope with a heavy load.

#### test 2 - car test
This test sets up a short course for a car with a motorized joint, and various obstacles.
- use `wasd` controls to drive the car

#### test 3 - raycast test
This test demonstrates the raycast feature.

#### test 4 - fork test
This test demonstrates the `fork` feature, which is the ability of the engine to
quickly clone itself, and later "restore" the forked state.
- press `f` to fork the solver state at the current position
- press `r` to restore the forked state, "rewinding" time.

#### test 5 - perf test
This test creates a 10x10 stack of boxes, and times how long it takes to run 5000 steps.
It logs the result to the console.

#### test 6 - particle test
This test demonstrates "particle" support. Particles are bodies in the engine which can
bounce off other bodies, but don't themselves take up any space or impart any forces.

#### test 7 - self-righting test
This test is a simulation of a one-wheeled vehicle. It uses a PID controller to translate
player input to motor impulses that steer the vehicle without tipping.
- use `wasd` controls to drive the vehicle

#### test 8 - sensor test
This test demonstrates sensor shapes. The outlined shapes are the sensors, and will change
color when detecting an overlap.
- use `wasd` controls to move the shape with the sensors around.

#### test 9 - continuous collision detection test
This test demonstrates continuous collision detection. It defaults to TOI off,
- press `t` to toggle time of impact.
