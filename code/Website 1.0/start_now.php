<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Read Entire File and Output Lines</title>
</head>
<body>

<?php

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
            echo "<p>$nextLine</p>";
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