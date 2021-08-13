# Composite Test Object for Simulated Robotic Manipulation Tasks
This repository contains code used to generate URDF models of a variety of test objects. The objects are built from two 3d-printed structures, M6 bolts and cylindrical weights. The idea is to make it easy to produce real objects that are very similar to the simulated ones and that can be arranged in a variety of configurations depending on where the weights are placed.

![CompositeTestObjectRenderingCropped](https://user-images.githubusercontent.com/10478385/129379216-9d5654d4-48e8-4de1-a522-49244e420ea1.png)

## Usage
The python script takes as arguments the locations of the bolts and weights and returns as the output the name of the generated URDF, the mass, the centre of mass and the inertia matrix of the object.
```
usage: GenerateObjectConfiguration.py [-h] [-b [POSITION [POSITION ...]]] [-s [POSITION [POSITION ...]]] [-p [POSITION [POSITION ...]]] [-o [FILE]] [-v]

Generate the URDF of a composite object with predictable inertial parameters.

optional arguments:
  -h, --help            show this help message and exit
  -b [POSITION [POSITION ...]], --bolts [POSITION [POSITION ...]]
                        List of bolt locations.
  -s [POSITION [POSITION ...]], --steelweights [POSITION [POSITION ...]]
                        List of weights locations where steel weights are placed.
  -p [POSITION [POSITION ...]], --plasticweights [POSITION [POSITION ...]]
                        List of weights locations where plastic weights are placed.
  -o [FILE], --output [FILE]
                        Optional filename of the output URDF to generate.
  -v, --version         show program's version number and exit
```

### Holes locations and numbering
The bolt holes (Bx) and weight holes (Wy) are referred using a number system that is based on the following chart. As the input, the script accepts numbers which refers to those in the chart, but no sanity-check is done.

![BoltsWeightsNumbering](https://user-images.githubusercontent.com/10478385/120645605-99adef80-c446-11eb-883a-d9bfffcadfc3.png)


### Example
```
phil@stars:~/composite-test-object$ python3 GenerateObjectConfiguration.py -b 0 14 -s 0 1 2 3 4 -o test.urdf
test.urdf
1.7036018760742466
[0.         0.01433761 0.025     ]
[[ 0.02142775  0.          0.        ]
 [ 0.          0.00509287 -0.00061134]
 [ 0.         -0.00061134  0.02374549]]
```

### Interesting Weight Patterns
A few patterns that can be made with the modular object might be particularly interesting. Note that the locations of the bolts are not specified because you will want to choose locations depending on your experimental setup.

```
Symbols: 
  Steel weight   (S): 0
  Plastic Weight (P): o
  No weight         : .

Patterns       Weights Locations

    0 0 0
      0        S: 0 1 2 3
      o        P: 4 5 6 8
      o
      o
    . o .
   "Hammer"

    0 0 0
      o        S: 0 1 2 7 8 9
      o        P: 3 4 5 6
      o
      o
    0 0 0
  "Barbell"

    0 0 0
      0        S: 0 1 2 3 4 5 6
      0
      0
      0
    . . .
    "Tee"

    0 0 0
      0        S: 0 1 2 3 4 5 6 7 8 9
      0
      0
      0
    0 0 0
"Homogeneous" 

    0 . 0
      .        S: 0 2 7 9
      .
      .
      .
    0 . 0
  "Corners"

    . 0 .
      0        S: 1 3 4 5 6 8
      0
      0
      0
    . 0 .
    "Rod" 

    0 0 0
      0        S: 0 1 2 3 4
      0        P: 5 6 7 8 9
      o
      o
    o o o
"Half-N-Half" 
```
To generate these objects, the following command lines can be used (bolts locations are not specified):
```
$ ./GenerateObjectConfiguration.py -s 0 1 2 3 -p 4 5 6 8 -o Hammer.urdf
$ ./GenerateObjectConfiguration.py -s 0 1 2 7 8 9 -p 3 4 5 6 -o Barbell.urdf
$ ./GenerateObjectConfiguration.py -s 0 1 2 3 4 5 6 -o Tee.urdf
$ ./GenerateObjectConfiguration.py -s 0 1 2 3 4 5 6 7 8 9 -o Homogeneous.urdf
$ ./GenerateObjectConfiguration.py -s 0 2 7 9 -o Corners.urdf
$ ./GenerateObjectConfiguration.py -s 1 3 4 5 6 8 -o Rod.urdf
$ ./GenerateObjectConfiguration.py -s 0 1 2 3 4 -p 5 6 7 8 9 -o Half-N-Half.urdf
```


## Plastic Half-structure
Inside the structure (left) and outside the structure (right). Large holes and used to host the weights and smaller holes are used to host the bolts. Both halves are alike and the space used to fit the head of the M6 bolt can also be used to fit a hex nut.

<img src="https://user-images.githubusercontent.com/10478385/120525548-39677100-c3a6-11eb-9483-9c053e8e0814.png" alt="Inside of Structure" height="400"/>
<img src="https://user-images.githubusercontent.com/10478385/120525557-3bc9cb00-c3a6-11eb-8546-acb9feadb68c.png" alt="Outside of Structure" height="400"/>

When printing the structure using PLA with a 20% infill, the weight of each half-structure should be about 320 grams.

## Weights
The cylindrical weights have a diameter of 1 inch (25.4mm) and a height of 0.5 inch (12.7mm). Two kits (20 weights) were bought from [here](https://www.amazon.ca/Precision-Calibration-Digital-Balance-Jewellery/dp/B082MMGG92) and pairs of two 50 grams weights are attached together such that each cylindrical steel weight (with height 1 inch) is 100 grams (mass density of about 7.77 g/cm^3 or 7770 Kg/m^3).

Similarly sized weights can easily be made out of wood, plastic or foam rods, providing a variety of weights. For instance, an ABS rod was bought from [here](https://www.mcmaster.com/8587K6/) and cut into pieces to make several 17 grams weights.

## Bolts
The two halves of the structures are meant to be held together with bolts, holding the weights in between. The diameter of the bolt holes is 6.6 mm and the space for the head of the bolts has a diameter of 11.8 mm and a depth of 6 mm. The distance between two vertices of the hex nut being 11.547 mm, some clearance is left to allow the nut to turn and a small tool to hold it while a bolt is being placed.
- [Bolts](https://www.mcmaster.com/91290A203/)
- [Nuts](https://www.mcmaster.com/90592A016/)
