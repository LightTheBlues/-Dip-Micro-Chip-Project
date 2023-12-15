<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Read Entire File and Output Lines</title>
</head>
<body>

<?php
    $timeStamp = array();
    $mode = array();
    $ID = array();
    $DLC = array();
    $DATA_0 = array();
    $DATA_1 = array();
    $DATA_2 = array();
    $DATA_3 = array();
    $DATA_4 = array();
    $DATA_5 = array();
    $DATA_6 = array();
    $DATA_7 = array();
	
// Specify the path to your text file
$filePath = 'CanBusFile.txt';

// Open the file for reading
$fileHandle = fopen($filePath, 'r');

if ($fileHandle) {
    // Find the position of the second instance of "//---------------------------------"
    $secondInstancePos = 0;
    $foundInstances = 0;

    while (($line = fgets($fileHandle)) !== false) {
        if (strpos($line, "//---------------------------------") !== false) {
            $foundInstances++;

            if ($foundInstances === 2) {
                break;
            }
        }
        $secondInstancePos = ftell($fileHandle);
    }

    // Output one line after the second instance
    if ($foundInstances === 2) {
        $lineAfterSecondInstance = fgets($fileHandle);
        echo "<p>$lineAfterSecondInstance</p>";

        // Continue reading and outputting lines
        while (($nextLine = fgets($fileHandle)) !== false) {
            // Output the next line
			$nextLine = str_replace(';', '    ', $nextLine);
            echo "<p>$nextLine</p>";
			
			
    $elements = explode(';', $nextLine);

    // Loop through the elements and populate the arrays
    for ($i = 0; $i < count($elements); $i++) {
        switch ($i) {
            case 0:
                $timeStamp[] = $elements[$i];
                break;
            case 1:
                $mode[] = $elements[$i];
                break;
            case 2:
                $ID[] = $elements[$i];
                break;
            case 3:
                $DLC[] = $elements[$i];
                break;
            case 4:
                $DATA_0[] = $elements[$i];
                break;
            case 5:
                $DATA_1[] = $elements[$i];
                break;
            case 6:
                $DATA_2[] = $elements[$i];
                break;
            case 7:
                $DATA_3[] = $elements[$i];
                break;
            case 8:
                $DATA_4[] = $elements[$i];
                break;
            case 9:
                $DATA_5[] = $elements[$i];
                break;
            case 10:
                $DATA_6[] = $elements[$i];
                break;
            case 11:
                $DATA_7[] = $elements[$i];
                break;
            // Add more cases if needed
        }
    }
        }
    } else {
        echo "<p>No line found after the second instance.</p>";
    }


    // Close the file handle
    fclose($fileHandle);

} else {
    echo "<p>Failed to open the file.</p>";
}

?>

</body>
</html>