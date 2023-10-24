# Brief

This code was submitted to the [GPPC^2](https://gppc.search-conference.org/) 2023 competition for the anyangle track.
The details for this submission are as follows:
- Submission name: RayScan
- Submitters: Ryan Hechenberger
- Affiliation: Monash University
- Country: Australia
- Short Description:

	An any-angle variant of the Euclidean pathfinding algorithm, RayScan with the bypass extension.
    Is fully online and finds the shortest path on the grid.
    Used some utility code from original implementation; however, RayScan itself is completely redone.

## Citation

If you find this code useful, the participant has asked to cite the following paper.

	@article{RayScan_ICAPS2020,
	title={Online Computation of {E}uclidean Shortest Paths in Two Dimensions},
	volume={30},
	url={https://ojs.aaai.org/index.php/ICAPS/article/view/6654},
	DOI={10.1609/icaps.v30i1.6654},
	number={1},
	journal={Proceedings of the International Conference on Automated Planning and Scheduling},
	author={Hechenberger, Ryan and Stuckey, Peter J and Harabor, Daniel and Le Bodic, Pierre and Cheema, Muhammad Aamir},
	year={2020},
	pages={134--142}
	}

## Evaluation Statistics

	Queries Solved: 256000
	Optimally Solved: 256000
	Sum Query Time: 664.235066619s
	Sum Preprocess Time: 0.0s
	Preprocess Storage: 0kB
	Max RAM: 51MB
	Avg Path Length: 1001.8031454093864
	Avg Subopt: -
	Undominated: Yes

For a full comparison of this submission against others in the competition, please see the GPPC website.

# Licensing

This code uses the GPPC startkit licensed under MIT found in `LICENSE.gppc`.

All code written by the participant is licensed under an open source license of their specification, outlined in `LICENSE`.
It is the responsibility of the participant to ensure they have fulfilled all licensing requirements for their submission and any included libraries.
