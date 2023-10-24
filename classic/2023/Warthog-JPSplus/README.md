# Breif

This code was submitted to the [GPPC^2](https://gppc.search-conference.org/) 2023 competition for the classic track.
The details for this submission are as follows:
- Submission name: Warthog-JPS+
- Submitters: Ryan Hechenberger
- Affiliation: Monash University
- Country: Australia
- Short Description:

	Warthog JPS+ implementation, fast optimal offline pathfinding.
	Adapted to work with the competition startkit, but otherwise unmodified and as implemented by the original authors.
	JPS2+ was not used due to a bug in Warthog at the time.

## Citation

If you find this code useful, the participant has asked to cite the following paper.

	@article{Harabor_Grastien_2014,
	title={Improving Jump Point Search},
	volume={24},
	url={https://ojs.aaai.org/index.php/ICAPS/article/view/13633},
	DOI={10.1609/icaps.v24i1.13633},
	number={1},
	journal={Proceedings of the International Conference on Automated Planning and Scheduling},
	author={Harabor, Daniel and Grastien, Alban},
	year={2014},
	pages={128--135}
	}

## Evaluation Statistics

	Queries Solved: 256000
	Optimally Solved: 256000
	Sum Query Time: 134.34256915s
	Sum Preprocess Time: 0.002317598s
	Preprocess Storage: 2GB
	Max RAM: 642MB
	Avg Path Length: 1062.564496597453
	Avg Subopt: -
	Undominated: Yes

For a full comparison of this submission against others in the competition, please see the GPPC website.

# Licensing

This code uses the GPPC startkit licensed under MIT found in `LICENSE.gppc`.

All code written by the participant is licensed under an open source license of their specification, outlined in `LICENSE`.
It is the responsibility of the participant to ensure they have fulfilled all licensing requirements for their submission and any included libraries.
