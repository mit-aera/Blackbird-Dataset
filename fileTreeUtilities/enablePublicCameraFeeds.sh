find BlackbirdDatasetData/ -path '*Depth*' -type f | parallel --progress aws s3api put-object-acl --bucket ijrr-blackbird-dataset --key '{}' --acl private
