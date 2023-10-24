# Breif

This code was submitted to the [GPPC^2](https://gppc.search-conference.org/) 2023 competition for the classic track.
The details for this submission are as follows:
- Submission name: 2013-Dao
- Submitters: Shizhe Zhao and Ryan Hechenberger
- Affiliation: Monash University
- Country: Australia
- Short Description:

	The original implementation from Nathan Sturtevant from GPPC-2013.
    An online suboptimal pathfinding algorithm.
    Adapted by Shizhe Zhao and Ryan Hechenberger for the updated GPPC-2013 competition.
    Changes were made to make it compatible with the larger Iron Harvest maps.
    Further potential speedups could be obtained by increasing the region size from 16x16, though would require significant changes in the codebase to support this.

## Citation

If you find this code useful, the participant has asked to cite the following paper.

	@article{Sturtevant_2007,
	title={Memory-Efficient Abstractions for Pathfinding},
	journal={Proceedings of the AAAI Conference on Artificial Intelligence and Interactive Digital Entertainment},
	volume={3},
	number={1},
	url={https://ojs.aaai.org/index.php/AIIDE/article/view/18778},
	DOI={10.1609/aiide.v3i1.18778},
	author={Sturtevant, Nathan},
	year={2007},
	pages={31--36}
	}

## Evaluation Statistics

	Queries Solved: 256000
	Optimally Solved: 3344
	Sum Query Time: 822.998263805s
	Sum Preprocess Time: 0.0s
	Preprocess Storage: 0kB
	Max RAM: 298MB
	Avg Path Length: 1098.9213389245538
	Avg Subopt: 1.0563158593750002
	Undominated: No

For a full comparison of this submission against others in the competition, please see the GPPC website.

# Licensing

This code uses the GPPC startkit licensed under MIT found in `LICENSE.gppc`.

All code written by the participant is licensed under an open source license of their specification, outlined in `LICENSE`.
It is the responsibility of the participant to ensure they have fulfilled all licensing requirements for their submission and any included libraries.
