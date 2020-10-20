# k-Shortest Paths with limited Overlap

The code in this repository was used in the following publications (please cite):

- Theodoros Chondrogiannis, Panagiotis Bouros, Johann Gamper, Ulf Leser, and David B. Blumenthal, 
Finding k-shortest paths with limited overlap.
The VLDB Journal 29(5) (2020)

- Theodoros Chondrogiannis, Panagiotis Bouros, Johann Gamper, and Ulf Leser,
Exact and Approximate Algorithms for Finding k-Shortest Paths with Limited Overlap ,
In Proc. of the 20th Int. Conf. on Extending Database Technology (EDBT) (2017) 

- Theodoros Chondrogiannis, Panagiotis Bouros, Johann Gamper, and Ulf Leser,
Alternative Routing: k-Shortest Paths with Limited Overlap ,
In Proc. of the 23rd ACM SIGSPATIAL Int. Conf. on Advances in Geographic Information Systems (GIS) (2015)

## Algorithms

The following table illustrates the inplemented algorithms:

| Abbreviation | Algorithm |
| ------ | ------ |
| op | The OnePass algorithm |
| mp | The MultiPass algorithm |
| opplus | The OnePass+ performance-oriented heuristic algorithm |
| svpplus | The SVP+ performance-oriented heuristic algorithm |
| esx | The ESX performance-oriented heuristic algorithm |
| svp-c | The SVP-C completeness-oriented heuristic algorithm |
| esx-c | The ESX-C completeness-oriented heuristic algorithm |

## Tests

In order to run the program and test the algorithms you must first compile the source code. Before compliling, change the compiler the the paths in the Makefile. Then, in order to compile, simply

```sh
$ make
```

Then, in order to run the run.exec with the sample file:

```sh
$ ./run.exec -f sample/sample.gr -k [PATHS] -t [THRESHOLD] -s [SOURCE] -d [DESTINATION] -a [ALGORITHM]
```
The following table shows the possible values for each parameter.

| Parameter | Description | Values |
| --- | --- | --- |
| PATHS | The number of requested result paths k | [0,+inf] |
| THRESHOLD | Similarity threshold θ | [0,1] |
| SOURCE | The source query node | [0,NUM_NODES] |
| DESTINATION | The target query node | [0,NUM_NODES] |
| ALGORITHM | The selected algorithm | op\|mp\|opplus\|svpplus\|esx\|svp-c\|esx-c |

## License

This work is licensed under MIT License.
