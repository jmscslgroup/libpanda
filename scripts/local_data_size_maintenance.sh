##AUTHORS: Gracie Gumm, Matthew Nice
#!/bin/bash
VIN=$(cat /etc/libpanda.d/vin)
local_path=/var/panda/CyverseData/JmscslgroupData/PandaData
FILE=/home/circles/libpanda/scripts/upload_status.csv

# creating index file if it does not already exist
if [ -f "$FILE" ]; then
  echo "$(basename $FILE) exists..."
else
  echo "Name, Date, Time, Type, Size (Bytes), Status, Last Checked" >> "$FILE" #Add in time checked
  echo "Creating index file (upload_status.csv)..."
fi

local_files=`ls -R $local_path`
for file in $local_files; do
  name=$(basename $file)
  
  # Ignore the log.csv file
  if [[ "$name" == "upload_status.csv" ]]; then
    continue
  fi
  
  # Check if the file has already been added to log.csv and update status if uploaded now
  if grep -q "^$(basename $file)," $FILE; then
    previous_status=$(grep "$name" "$FILE" | awk -F', ' '{print $6}')
    if [ "$previous_status" = "Not Uploaded" ]; then
      local_size=$(grep "$name" "$FILE" | awk -F', ' '{print $5}')
      date_directory=$(grep "$name" "$FILE" | awk -F', ' '{print $2}')
      time_file=$(grep "$name" "$FILE" | awk -F', ' '{print $3}')
      type=$(grep "$name" "$FILE" | awk -F', ' '{print $4}')
      remote_files=$(ils -r /iplant/home/sprinkjm/private-circles/$VIN/libpanda/$date_directory/) 
      remote_found=0
      for remote_file in $remote_files; do
        remote_basename=$(basename "$remote_file")
        if [[ "$name" == "$remote_basename" ]]; then
	##TODO ACCOUNT FOR SWITCH TO NDD DATASTORE
          remote_size_str=$(ils -l "/iplant/home/sprinkjm/private-circles/$VIN/libpanda/$date_directory/$remote_basename" | awk '{printf$4}')
          len_str=$(( ${#remote_size_str} / 2))
          remote_size=${remote_size_str:0:$len_str}
          if [[ "$local_size" == "$remote_size" ]]; then
            status="Uploaded"
            time_check=$(date '+%Y-%m-%d %H:%M:%S')
            sed -i "/^$name,/d" $FILE
            echo "$name, $date_directory, $time_file, $type, $local_size, $status, $time_check" >> "$FILE"
            remote_found=1
            break
          fi
        fi
      done
      if [[ "$remote_found" == "0" ]]; then
        status="Not Uploaded"
        time_check=$(date '+%Y-%m-%d %H:%M:%S')
        sed -i "/^$name,/d" $FILE
        echo "$name, $date_directory, $time_file, $type, $local_size, $status, $time_check" >> "$FILE"
      fi
    fi
  # Add files to log for the first time
  else
    if [[ $file == *.csv ]]; then 
      year_file=${name:0:4}
      month_file=${name:5:2}
      day_file=${name:8:2}
      time_file=${name:11:8}
      date_directory=$year_file"_"$month_file"_"$day_file
      type=${name:38:3}
	##TODO ACCOUNT FOR SWITCH TO NDD DATASTORE
      remote_loc=`ils -r /iplant/home/sprinkjm/private-circles/$VIN/libpanda/$date_directory`
      local_size=$(du --bytes /var/panda/CyverseData/JmscslgroupData/PandaData/$date_directory/$name | awk '{print $1}')
      if [ -z "$remote_loc" ]; then
        status="Not Uploaded"
        time_check=$(date '+%Y-%m-%d %H:%M:%S')
      else
        status="Not Uploaded"
        time_check=$(date '+%Y-%m-%d %H:%M:%S')
        remote_files=$(ils -r /iplant/home/sprinkjm/private-circles/$VIN/libpanda/$date_directory/)
        for remote_file in $remote_files; do
          remote_basename=$(basename "$remote_file")
          if [[ "$name" == "$remote_basename" ]]; then
	##TODO ACCOUNT FOR SWITCH TO NDD DATASTORE
            remote_size_str=$(ils -l "/iplant/home/sprinkjm/private-circles/$VIN/libpanda/$date_directory/$remote_basename" | awk '{printf$4}')
            len_str=$(( ${#remote_size_str} / 2))
            remote_size=${remote_size_str:0:$len_str}
            if awk -v local="$local_size" -v remote="$remote_size" 'BEGIN { if (local >= remote) exit 0; exit 1 }'; then
              status="Uploaded"
              time_check=$(date '+%Y-%m-%d %H:%M:%S')
              echo "$name is properly uploaded"
            fi
            break
            status="Not Uploaded"
            echo "$name is NOT uploaded"
            time_check=$(date '+%Y-%m-%d %H:%M:%S')
            break
          fi
        done
      fi
      echo "$name, $date_directory, $time_file, $type, $local_size, $status, $time_check" >> "$FILE"
    fi
  fi
done

# DELETE SECTION
critical_storage=50
used_storage_float=$(lsblk -fmo NAME,FSUSE%,FSAVAIL,FSSIZE | grep 'mmcblk0p2' | awk '{printf "%.1f", $2 }')
used_storage=${used_storage_float%.*}
delete_to=20
echo $used_storage_float
if [[ $used_storage -gt $critical_storage ]]; then
  echo "Memory usage critical... deleting files."
  while [[ $used_storage -gt $delete_to ]]; do
    file_name=$(awk -F ',' 'NR==2{print $1}' $FILE)
    name=$(basename "$file_name")
    local_size=$(grep "$name" "$FILE" | awk -F', ' '{print $5}')
    previous_status=$(grep "$name" "$FILE" | awk -F', ' '{print $6}')
    date_directory=$(grep "$name" "$FILE" | awk -F', ' '{print $2}')
    time_file=$(grep "$name" "$FILE" | awk -F', ' '{print $3}')
    type=$(grep "$name" "$FILE" | awk -F', ' '{print $4}')

    # Check that log.csv file is not empty.
    if [ "$(wc -l < "$FILE")" -lt 2 ]; then
      echo "Error in deleting files: no files are available to be deleted"
      break
    fi
    # Delete files if status is uploaded. Double check status
    if [ "$previous_status" = "Uploaded" ]; then
      echo "$name has been uploaded and will be deleted"
	##TODO ACCOUNT FOR SWITCH TO NDD DATASTORE
      remote_files=$(ils -r /iplant/home/sprinkjm/private-circles/$VIN/libpanda/$date_directory/) 
      remote_found=0
      for remote_file in $remote_files; do
        remote_basename=$(basename "$remote_file")
        if [[ "$name" == "$remote_basename" ]]; then
	##TODO ACCOUNT FOR SWITCH TO NDD DATASTORE
          remote_size_str=$(ils -l "/iplant/home/sprinkjm/private-circles/$VIN/libpanda/$date_directory/$remote_basename" | awk '{printf$4}')
          len_str=$(( ${#remote_size_str} / 2))
          remote_size=${remote_size_str:0:$len_str}
          # FILE IS UPLOADED, GOOD TO DELETE
          if [[ "$local_size" == "$remote_size" ]]; then
            status="Uploaded"
            time_check=$(date '+%Y-%m-%d %H:%M:%S')
            echo "Deleting $name..."
            sudo rm "${local_path}/${date_directory}/${file_name}"
            sed -i '2d' $FILE
	    echo "Storage is currently: $used_storage"
            remote_found=1
            break
          fi
        fi
      done
      if [[ "$remote_found" == "0" ]]; then
        status="Not Uploaded"
        time_check=$(date '+%Y-%m-%d %H:%M:%S')
        sed -i "/^$name,/d" $FILE
        echo "$name, $date_directory, $time_file, $type, $local_size, $status, $time_check" >> "$FILE"
      fi  
    else
      status="Not Uploaded"
      time_check=$(date '+%Y-%m-%d %H:%M:%S')
      sed -i "/^$name,/d" $FILE
      echo "$name, $date_directory, $time_file, $type, $local_size, $status, $time_check" >> "$FILE"
    fi
    used_storage_float=$(lsblk -fmo NAME,FSUSE%,FSAVAIL,FSSIZE | grep 'mmcblk0p2' | awk '{printf "%.1f", $2 }')
    used_storage=${used_storage_float%.*}
  done
  echo "Deletion process is completed. Currrent used storage $used_storage_float%"
fi
