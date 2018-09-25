# The Blackbird Dataset: A large-scale dataset for UAV perception in aggressive flight

<!-- [![Video Link](https://img.youtube.com/vi/_VBww8YQuA8/0.jpg)](https://www.youtube.com/watch?v=_VBww8YQuA8) -->

The Blackbird Dataset was created by the [AgileDrones group](http://agiledrones.mit.edu) at the [MIT FAST Lab](http://karaman.mit.edu/group.html) and will be published in the proceedings of ISER 2018.

## Preview the Dataset



## Download the Dataset

```bash
# Clone this repo into your dataset destination folder
git clone https://github.com/AgileDrones/Blackbird-Dataset.git
cd Blackbird-Dataset
mkdir data

# Install python dependencies for included utilities
pip install -r requirements.txt

# EX: download subset of the dataset via HTTP/BitTorrent hybrid method (fastest)
./downloaderUtility.py downloadSubset flights='(trajectory in ["tiltedThrice"] and topSpeed >= 6.0)' files='["videoPreview", "bagfile"]' data/

# EX: download all preview videos of the dataset via HTTP/BitTorrent hybrid method (fastest).
# WARNING: total size of preview videos is 69GB. If you'd like to preview the dataset, please refer to the table above.
./downloaderUtility.py downloadSubset files='["videoPreview"]' data/


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