#!/usr/bin/env ruby

# ffmpeg-mosaic.rb
# Author: William Woodruff
# ------------------------
# Create an NxM "mosaic" of videos using ffmpeg and a generated complex filter.
# Requires ffmpeg.
# ------------------------
# This code is licensed by William Woodruff under the MIT License.
# http://opensource.org/licenses/MIT

throttle = !!ARGV.delete("--throttle")
shortest = !!ARGV.delete("--shortest") ? 1 : 0

width, height = ARGV.shift&.split("x", 2)&.map(&:to_i)

columns, rows = ARGV.shift&.split("x", 2)&.map(&:to_i)

output = ARGV.shift

inputs = ARGV

abort("Please add another row or column.") if rows * columns < inputs.size

abort "One or more invalid inputs." unless !inputs.empty? && inputs.all? { |i| File.file?(i) }

pane_width = width / columns
pane_height = height / rows
pane_dim = "#{pane_width}x#{pane_height}"

args = ["ffmpeg", "-y"]

inputs.each_with_index do |input, _idx|
#  if _idx != 3
#    args.concat ["-ss", "00:00:03","-i", input] # "-threads:#{_idx}", "1"]
#  else
    args.concat ["-i", input]
#  end
end

args.concat ["-threads", "6"] if throttle

input_mosaic = inputs.each_slice(columns)

args << "-filter_complex"

filter = ""
filter << "color=size=#{width}x#{height}:c=black [base];\n"

input_mosaic.each_with_index do |row, i|
  row.each_with_index do |col, j|
    idx = (i * columns) + j
    filter << "[#{idx}:v] setpts=PTS-STARTPTS, scale=#{pane_dim} [a#{idx}];\n"
  end
end

tmp = "tmp0"
filter << "[base][a0] overlay=shortest=#{shortest}:eof_action=pass [#{tmp}];\n"

input_mosaic.each_with_index do |row, i|
  row.each_with_index do |col, j|
    next if i.zero? && j.zero? # this one comes for free, see above
    idx = (i * columns) + j
    tmp_next = "tmp#{idx}"
    xy = ":x=#{pane_width * j}:y=#{pane_height * i}"

    filter << if idx != inputs.size - 1
                "[#{tmp}][a#{idx}] overlay=shortest=#{shortest}:eof_action=pass#{xy} [#{tmp_next}];\n"
              else # last input has special syntax
                "[#{tmp}][a#{idx}] overlay=shortest=#{shortest}:eof_action=pass#{xy},fps=fps=60"
              end

    tmp = tmp_next
  end
end

# -an disables audio output, since it doesn't make much sense here
args.concat [filter, "-an", output]

#abort(args.join(" "))

exec(*args)
