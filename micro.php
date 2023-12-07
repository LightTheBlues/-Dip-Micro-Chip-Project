<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>MicroDip</title>
    <style>
    
    #menu-toggle {
            display: none;
        }

        .menu-icon {
            cursor: pointer;
            display: block;
            padding: 10px;
            background-color: #333;
            color: #fff;
            text-align: center;
        }

        #menu {
            display: none;
            position: absolute;
            top: 60px;
            right: 20px;
            background-color: #1e1e1e;
            padding: 10px;
            border-radius: 5px;
            box-shadow: 0 0 10px rgba(0, 0, 0, 0.2);
            z-index: 1;
        }

        #menu a {
            display: block;
            color: #fff;
            padding: 10px;
            text-decoration: none;
            font-size: 16px;
            margin-bottom: 5px;
        }
    
    
        body {
            font-family: 'Arial', sans-serif;
            background-color: #1e1e1e;
            color: #ffffff;
            margin: 0;
            padding: 0;
            text-align: center;
        }

        header {
            background-color: #333;
            padding: 20px;
        }

        h1 {
            color: #00bcd4;
        }

        section {
            padding: 40px;
        }

        h2 {
            color: #ff6666; /* Light red color */
            margin-bottom: 20px;
        }

        footer {
            background-color: #333;
            padding: 20px;
        }

        p {
            font-size: 18px;
            line-height: 1.6;
        }

        .techy-button {
            background-color: #00bcd4;
            color: #ffffff;
            padding: 10px 20px;
            font-size: 16px;
            text-decoration: none;
            border-radius: 5px;
            display: inline-block;
            margin-top: 20px;
        }
    </style>
</head>
<body>

    <header>
        <h1>MicroDip Team - CAN Communication</h1>
        <div class="menu-icon" onclick="toggleMenu()">&#9776; Menu</div>
        <input type="checkbox" id="menu-toggle">
        <div id="menu">
            <a href="micro.php">Home</a>
            <a href="testing.html">Start Now</a>
        </div>
    </header>

    <section>
        <h2>Background</h2>
        <p>A bit about the project...</p>

        <h2>Our Goal</h2>
        <p>Our aim is to...</p>

        <h2>How to Use</h2>
        <p>Explain what this page analyzes, how to set up, and how to interpret results</p>
        <a href="https://github.com/LightTheBlues/uDip-Micro-Chip-Project" class="techy-button">Learn More</a>
    </section>

    <footer>
        <p>&copy; <?php echo date('Y'); ?> Andrew, Bryan, Jun, Leo, Yeong</p>
    </footer>
    <script>
        function toggleMenu() {
            var menu = document.getElementById("menu");
            menu.style.display = (menu.style.display === "block") ? "none" : "block";
        }
    </script>

</body>
</html>