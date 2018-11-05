# The Blackbird Dataset: A large-scale dataset for UAV perception in aggressive flight

<!-- [![Video Link](https://img.youtube.com/vi/_VBww8YQuA8/0.jpg)](https://www.youtube.com/watch?v=_VBww8YQuA8) -->

The Blackbird Dataset was created by the [AgileDrones group](http://agiledrones.mit.edu) at the [MIT FAST Lab](http://karaman.mit.edu/group.html) and has been published in the proceedings of ISER 2018 [(arXiv link)](https://arxiv.org/abs/1810.01987).

**NOTE: The BlackbirdDataset will be in prerelease until its official release at ISER 2018 on November 5th.**

## Preview the Dataset

| Yaw Forward Trajectories<br>*Click for preview video*                                        |||||||||
| :-----------: | :------: | :------: | :------: | :------: | :------: | :------: | :------: | :------: | 
| Top speed (m/s) |   0.5  |   1.0    |   2.0    |   3.0    |   4.0    |   5.0    |   6.0    |   7.0    |
|  3D Figure 8  |    \-    |     \-   |    \-    |    \-    |    \-    |    \-    |    \-    |    \-    |
|   Ampersand   |    \-    | [✓][b11] | [✓][b21] |    \-    |    \-    |    \-    |    \-    |    \-    |
|   Bent Dice   | [✓][c01] | [✓][c11] | [✓][c21] | [✓][c31] |    \-    |    \-    |    \-    |    \-    |
|    Clover     | [✓][d01] | [✓][d11] | [✓][d21] | [✓][d31] | [✓][d41] | [✓][d51] |    \-    |    \-    |
|     Dice      |    \-    | [✓][e11] | [✓][e21] | [✓][e31] |    \-    |    \-    |    \-    |    \-    |
| Flat Figure 8 |    \-    |    \-    |    \-    |    \-    |    \-    |    \-    |    \-    |    \-    |
|   Half-Moon   |    \-    | [✓][g11] | [✓][g21] | [✓][g31] | [✓][g41] |    \-    |    \-    |    \-    |
|     Mouse     | [✓][h01] | [✓][h11] | [✓][h21] | [✓][h31] | [✓][h41] | [✓][h51] | [✓][h61] | [✓][h71] |
|     Oval      |    \-    | [✓][i11] | [✓][i21] | [✓][i31] | [✓][i41] |    \-    |    \-    |    \-    |
|    Patrick    | [✓][j01] | [✓][j11] | [✓][j21] | [✓][j31] | [✓][j41] |    \-    |    \-    |    \-    |
|    Picasso    | [✓][k01] | [✓][k11] |    \-    | [✓][k31] | [✓][k41] | [✓][k51] |    \-    |    \-    |
|      Sid      | [✓][l01] | [✓][l11] | [✓][l21] | [✓][l31] | [✓][l41] |    \-    |    \-    |    \-    |
|    Sphinx     |    \-    | [✓][m11] | [✓][m21] | [✓][m31] | [✓][m41] |    \-    |    \-    |    \-    |
|     Star      | [✓][n01] | [✓][n11] | [✓][n21] | [✓][n31] | [✓][n41] | [✓][n51] |    \-    |    \-    |
|    Thrice     | [✓][o01] | [✓][o11] | [✓][o21] | [✓][o31] | [✓][o41] | [✓][o51] | [✓][o61] |    \-    |
| Tilted Thrice | [✓][p01] | [✓][p11] | [✓][p21] | [✓][p31] | [✓][p41] | [✓][p51] | [✓][p61] |    \-    |
|    Winter     | [✓][q01] |    \-    | [✓][q21] | [✓][q31] | [✓][q41] |    \-    |    \-    |    \-    |


| Constant Yaw Trajectories<br>*Click for preview video*                                        |||||||||
| :-----------: | :------: | :------: | :------: | :------: | :------: | :------: | :------: | :------: | 
| Top speed (m/s) |   0.5  |   1.0    |   2.0    |   3.0    |   4.0    |   5.0    |   6.0    |   7.0    |
|  3D Figure 8  |    ✓*    |     ✓*   |    ✓*    |    ✓*    |    ✓*    |    ✓*    |    \-    |    \-    |
|   Ampersand   |    \-    | [✓][b10] | [✓][b20] | [✓][b30] |    \-    |    \-    |    \-    |    \-    |
|   Bent Dice   |    \-    | [✓][c10] | [✓][c20] | [✓][c30] | [✓][c40] |    \-    |    \-    |    \-    |
|    Clover     |    \-    | [✓][d10] | [✓][d20] | [✓][d30] | [✓][d40] | [✓][d50] | [✓][d60] |    \-    |
|     Dice      |    \-    |    \-    | [✓][e20] | [✓][e30] | [✓][e40] |    \-    |    \-    |    \-    |
| Flat Figure 8 |    ✓*    |    ✓*    |    ✓*    |    ✓*    |    \-    |    ✓*    |    \-    |    \-    |
|   Half-Moon   |    \-    | [✓][g10] | [✓][g20] | [✓][g30] | [✓][g40] |    \-    |    \-    |    \-    |
|     Mouse     |    \-    | [✓][h10] | [✓][h20] | [✓][h30] | [✓][h40] | [✓][h50] | [✓][h60] | [✓][h70] |
|     Oval      |    \-    |    \-    | [✓][i20] | [✓][i30] | [✓][i40] |    \-    |    \-    |    \-    |
|    Patrick    |    \-    | [✓][j10] | [✓][j20] | [✓][j30] | [✓][j40] | [✓][j50] |    \-    |    \-    |
|    Picasso    | [✓][k05] | [✓][k10] | [✓][k20] | [✓][k30] | [✓][k40] | [✓][k50] | [✓][k60] |    \-    |
|      Sid      |    \-    | [✓][l10] | [✓][l20] | [✓][l30] | [✓][l40] | [✓][l50] | [✓][l60] | [✓][l70] |
|    Sphinx     |    \-    | [✓][m10] | [✓][m20] | [✓][m30] | [✓][m40] |    \-    |    \-    |    \-    |
|     Star      |    \-    | [✓][n10] | [✓][n20] | [✓][n30] | [✓][n40] | [✓][n50] |    \-    |    \-    |
|    Thrice     |    \-    | [✓][o10] | [✓][o20] | [✓][o30] | [✓][o40] | [✓][o50] | [✓][o60] | [✓][o70] |
| Tilted Thrice |    \-    | [✓][p10] | [✓][p20] | [✓][p30] | [✓][p40] | [✓][p50] | [✓][p60] | [✓][p70] |
|    Winter     |    \-    | [✓][q10] | [✓][q20] | [✓][q30] | [✓][q40] | [✓][q50] |    \-    |    \-    |

\* Calibration flights for dynamics, no camera data available.


## Download the Dataset

```bash
# Clone this repo into your dataset destination folder
git clone https://github.com/AgileDrones/Blackbird-Dataset.git
cd Blackbird-Dataset
mkdir data

# Install python dependencies for included utilities
pip install -r requirements.txt

# EX: download subset of the dataset via HTTP/BitTorrent hybrid method (fastest)
./downloaderUtility.py downloadSubset flights='(trajectory in ["tiltedThrice"] and topSpeed >= 6.0)' files='["videoPreview", "bagfile"]' BlackbirdDatasetData/

# EX: download all preview videos of the dataset via HTTP/BitTorrent hybrid method (fastest).
# WARNING: total size of preview videos is 69GB. If you'd like to preview the dataset, please refer to the table above.
./downloaderUtility.py downloadSubset files='["videoPreview"]' BlackbirdDatasetData/


```
## Downloader Documentation

`./downloaderUtility.py downloadSubset flights='(python_boolean_expression)' files='[filetype_list]' <output_location>`

* Options for `files=[...]` (string):
    * `["videoPreview", "bagfile", "lcmLog", "Camera_L_Images","Camera_R_Images", "Camera_D_Images" ]`
    * NOTE: small supportive files (such as CSVs) will be automatically downloaded.

* Options for `flights=(...)` filter expression. Note: selects all flights by default.
    * `trajectory` (string) in `["3dFigure8", "ampersand", "bentDice", "clover", "dice", "figure8", "halfMoon", "mouse", "oval", "patrick", "picasso", "sid", "sphinx", "star", "thrice", "tiltedThrice", "winter"]`
    * `topSpeed` (m/s float) in `[0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]`
    * `difficulty` (string) in `["easy", "medium", "hard", "extreme"]`
    * `yawType` (string) in `["yawConstant", "yawForward"]`
    * `location` (string) in `["Ancient_Egypt_Museum_Room", "Small_Apartment", "Large_Apartment_Night_Near_Column", "Outdoor_Patio_Night", "Large_Apartment_Day_Near_Kitchen", "Large_Apartment_Night_Near_Couches", "Ancient_Asia_Museum_Room"]` # Useful for specifying environment to download for flights rendered in multiple environments.
    * `environment` (string) in `["Museum_Day", "Museum_Day_Small", "Butterfly_World", "Hazelwood_Loft_Full_Night", "Hazelwood_Loft_Full_Day", "NYC_Subway", "NYC_Subway_Station"]`

## Citation
If you find this work useful for your research, please cite:
```bibtex
@inproceedings{antonini2018blackbird,
  title={The Blackbird Dataset: A large-scale dataset for UAV perception in aggressive flight},
  author={Antonini, Amado and Guerra, Winter and Murali, Varun and Sayre-McCord, Thomas and Karaman, Sertac},
  booktitle={2018 International Symposium on Experimental Robotics (ISER)},
  year={2018}
}
```

<!-- PREVIEW LINKS BELOW  -->

<!-- Constant yaw trajectory preview links for table -->
[b10]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/ampersand/yawConstant/maxSpeed1p0/videos/
[b20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/ampersand/yawConstant/maxSpeed2p0/videos/
[b30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/ampersand/yawConstant/maxSpeed3p0/videos/

[c10]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/bentDice/yawConstant/maxSpeed1p0/videos/
[c20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/bentDice/yawConstant/maxSpeed2p0/videos/
[c30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/bentDice/yawConstant/maxSpeed3p0/videos/
[c40]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/bentDice/yawConstant/maxSpeed4p0/videos/

[d10]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/clover/yawConstant/maxSpeed1p0/videos/
[d20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/clover/yawConstant/maxSpeed2p0/videos/
[d30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/clover/yawConstant/maxSpeed3p0/videos/
[d40]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/clover/yawConstant/maxSpeed4p0/videos/
[d50]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/clover/yawConstant/maxSpeed5p0/videos/
[d60]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/clover/yawConstant/maxSpeed6p0/videos/

[e10]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/dice/yawConstant/maxSpeed1p0/videos/
[e20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/dice/yawConstant/maxSpeed2p0/videos/
[e30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/dice/yawConstant/maxSpeed3p0/videos/
[e40]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/dice/yawConstant/maxSpeed4p0/videos/

[g10]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/halfMoon/yawConstant/maxSpeed1p0/videos/
[g20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/halfMoon/yawConstant/maxSpeed2p0/videos/
[g30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/halfMoon/yawConstant/maxSpeed3p0/videos/
[g40]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/halfMoon/yawConstant/maxSpeed4p0/videos/

[h10]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawConstant/maxSpeed1p0/videos/
[h20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawConstant/maxSpeed2p0/videos/
[h30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawConstant/maxSpeed3p0/videos/
[h40]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawConstant/maxSpeed4p0/videos/
[h50]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawConstant/maxSpeed5p0/videos/
[h60]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawConstant/maxSpeed6p0/videos/
[h70]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawConstant/maxSpeed7p0/videos/

[i20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/oval/yawConstant/maxSpeed2p0/videos/
[i30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/oval/yawConstant/maxSpeed3p0/videos/
[i40]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/oval/yawConstant/maxSpeed4p0/videos/

[j10]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/patrick/yawConstant/maxSpeed1p0/videos/
[j20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/patrick/yawConstant/maxSpeed2p0/videos/
[j30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/patrick/yawConstant/maxSpeed3p0/videos/
[j40]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/patrick/yawConstant/maxSpeed4p0/videos/
[j50]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/patrick/yawConstant/maxSpeed5p0/videos/

[k05]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/picasso/yawConstant/maxSpeed0p5/videos/
[k10]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/picasso/yawConstant/maxSpeed1p0/videos/
[k20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/picasso/yawConstant/maxSpeed2p0/videos/
[k30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/picasso/yawConstant/maxSpeed3p0/videos/
[k40]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/picasso/yawConstant/maxSpeed4p0/videos/
[k50]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/picasso/yawConstant/maxSpeed5p0/videos/
[k60]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/picasso/yawConstant/maxSpeed6p0/videos/

[l10]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawConstant/maxSpeed1p0/videos/
[l20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawConstant/maxSpeed2p0/videos/
[l30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawConstant/maxSpeed3p0/videos/
[l40]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawConstant/maxSpeed4p0/videos/
[l50]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawConstant/maxSpeed5p0/videos/
[l60]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawConstant/maxSpeed6p0/videos/
[l70]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawConstant/maxSpeed7p0/videos/

[m10]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sphinx/yawConstant/maxSpeed1p0/videos/
[m20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sphinx/yawConstant/maxSpeed2p0/videos/
[m30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sphinx/yawConstant/maxSpeed3p0/videos/
[m40]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sphinx/yawConstant/maxSpeed4p0/videos/

[n10]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/star/yawConstant/maxSpeed1p0/videos/
[n20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/star/yawConstant/maxSpeed2p0/videos/
[n30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/star/yawConstant/maxSpeed3p0/videos/
[n40]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/star/yawConstant/maxSpeed4p0/videos/
[n50]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/star/yawConstant/maxSpeed5p0/videos/

[o10]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawConstant/maxSpeed1p0/videos/
[o20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawConstant/maxSpeed2p0/videos/
[o30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawConstant/maxSpeed3p0/videos/
[o40]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawConstant/maxSpeed4p0/videos/
[o50]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawConstant/maxSpeed5p0/videos/
[o60]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawConstant/maxSpeed6p0/videos/
[o70]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawConstant/maxSpeed7p0/videos/

[p10]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawConstant/maxSpeed1p0/videos/
[p20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawConstant/maxSpeed2p0/videos/
[p30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawConstant/maxSpeed3p0/videos/
[p40]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawConstant/maxSpeed4p0/videos/
[p50]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawConstant/maxSpeed5p0/videos/
[p60]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawConstant/maxSpeed6p0/videos/
[p70]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawConstant/maxSpeed7p0/videos/

[q10]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/winter/yawConstant/maxSpeed1p0/videos/
[q20]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/winter/yawConstant/maxSpeed2p0/videos/
[q30]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/winter/yawConstant/maxSpeed3p0/videos/
[q40]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/winter/yawConstant/maxSpeed4p0/videos/
[q50]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/winter/yawConstant/maxSpeed5p0/videos/

<!-- Yaw Forward trajectory preview links for table -->
[b11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/ampersand/yawForward/maxSpeed1p0/videos/
[b21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/ampersand/yawForward/maxSpeed2p0/videos/
[b31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/ampersand/yawForward/maxSpeed3p0/videos/

[c01]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/bentDice/yawForward/maxSpeed0p5/videos/
[c11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/bentDice/yawForward/maxSpeed1p0/videos/
[c21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/bentDice/yawForward/maxSpeed2p0/videos/
[c31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/bentDice/yawForward/maxSpeed3p0/videos/
[c41]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/bentDice/yawForward/maxSpeed4p0/videos/

[d01]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/clover/yawForward/maxSpeed0p5/videos/
[d11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/clover/yawForward/maxSpeed1p0/videos/
[d21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/clover/yawForward/maxSpeed2p0/videos/
[d31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/clover/yawForward/maxSpeed3p0/videos/
[d41]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/clover/yawForward/maxSpeed4p0/videos/
[d51]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/clover/yawForward/maxSpeed5p0/videos/
[d61]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/clover/yawForward/maxSpeed6p0/videos/

[e11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/dice/yawForward/maxSpeed1p0/videos/
[e21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/dice/yawForward/maxSpeed2p0/videos/
[e31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/dice/yawForward/maxSpeed3p0/videos/
[e41]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/dice/yawForward/maxSpeed4p0/videos/

[g11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/halfMoon/yawForward/maxSpeed1p0/videos/
[g21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/halfMoon/yawForward/maxSpeed2p0/videos/
[g31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/halfMoon/yawForward/maxSpeed3p0/videos/
[g41]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/halfMoon/yawForward/maxSpeed4p0/videos/

[h01]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawForward/maxSpeed0p5/videos/
[h11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawForward/maxSpeed1p0/videos/
[h21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawForward/maxSpeed2p0/videos/
[h31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawForward/maxSpeed3p0/videos/
[h41]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawForward/maxSpeed4p0/videos/
[h51]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawForward/maxSpeed5p0/videos/
[h61]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawForward/maxSpeed6p0/videos/
[h71]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/mouse/yawForward/maxSpeed7p0/videos/

[i11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/oval/yawForward/maxSpeed1p0/videos/
[i21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/oval/yawForward/maxSpeed2p0/videos/
[i31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/oval/yawForward/maxSpeed3p0/videos/
[i41]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/oval/yawForward/maxSpeed4p0/videos/

[j01]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/patrick/yawForward/maxSpeed0p5/videos/
[j11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/patrick/yawForward/maxSpeed1p0/videos/
[j21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/patrick/yawForward/maxSpeed2p0/videos/
[j31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/patrick/yawForward/maxSpeed3p0/videos/
[j41]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/patrick/yawForward/maxSpeed4p0/videos/
[j51]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/patrick/yawForward/maxSpeed5p0/videos/

[k01]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/picasso/yawForward/maxSpeed0p5/videos/
[k11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/picasso/yawForward/maxSpeed1p0/videos/
[k21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/picasso/yawForward/maxSpeed2p0/videos/
[k31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/picasso/yawForward/maxSpeed3p0/videos/
[k41]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/picasso/yawForward/maxSpeed4p0/videos/
[k51]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/picasso/yawForward/maxSpeed5p0/videos/
[k61]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/picasso/yawForward/maxSpeed6p0/videos/

[l01]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawForward/maxSpeed0p5/videos/
[l11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawForward/maxSpeed1p0/videos/
[l21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawForward/maxSpeed2p0/videos/
[l31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawForward/maxSpeed3p0/videos/
[l41]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawForward/maxSpeed4p0/videos/
[l51]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawForward/maxSpeed5p0/videos/
[l61]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawForward/maxSpeed6p0/videos/
[l71]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sid/yawForward/maxSpeed7p0/videos/

[m11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sphinx/yawForward/maxSpeed1p0/videos/
[m21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sphinx/yawForward/maxSpeed2p0/videos/
[m31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sphinx/yawForward/maxSpeed3p0/videos/
[m41]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/sphinx/yawForward/maxSpeed4p0/videos/

[n01]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/star/yawForward/maxSpeed0p5/videos/
[n11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/star/yawForward/maxSpeed1p0/videos/
[n21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/star/yawForward/maxSpeed2p0/videos/
[n31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/star/yawForward/maxSpeed3p0/videos/
[n41]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/star/yawForward/maxSpeed4p0/videos/
[n51]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/star/yawForward/maxSpeed5p0/videos/

[o01]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawForward/maxSpeed0p5/videos/
[o11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawForward/maxSpeed1p0/videos/
[o21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawForward/maxSpeed2p0/videos/
[o31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawForward/maxSpeed3p0/videos/
[o41]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawForward/maxSpeed4p0/videos/
[o51]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawForward/maxSpeed5p0/videos/
[o61]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawForward/maxSpeed6p0/videos/
[o71]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/thrice/yawForward/maxSpeed7p0/videos/

[p01]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawForward/maxSpeed0p5/videos/
[p11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawForward/maxSpeed1p0/videos/
[p21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawForward/maxSpeed2p0/videos/
[p31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawForward/maxSpeed3p0/videos/
[p41]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawForward/maxSpeed4p0/videos/
[p51]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawForward/maxSpeed5p0/videos/
[p61]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawForward/maxSpeed6p0/videos/
[p71]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/tiltedThrice/yawForward/maxSpeed7p0/videos/

[q01]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/winter/yawForward/maxSpeed0p5/videos/
[q11]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/winter/yawForward/maxSpeed1p0/videos/
[q21]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/winter/yawForward/maxSpeed2p0/videos/
[q31]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/winter/yawForward/maxSpeed3p0/videos/
[q41]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/winter/yawForward/maxSpeed4p0/videos/
[q51]: http://blackbird-dataset.mit.edu/BlackbirdDatasetData/winter/yawForward/maxSpeed5p0/videos/
