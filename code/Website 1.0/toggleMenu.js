// File: toggleMenu.js

document.addEventListener("DOMContentLoaded", function() {
    var menu = document.getElementById('menu');
    menu.style.display = 'none';

    function toggleMenu() {
        if (menu.style.display === 'block' || menu.style.display === '') {
            menu.style.display = 'none';
        } else {
            menu.style.display = 'block';
        }
    }
	
    // You can attach the event listener directly to the menu icon here if needed
    var menuIcon = document.querySelector('.menu-icon');
    if (menuIcon) {
        menuIcon.addEventListener('click', toggleMenu);
    }
});

function showContents(elem, ID) {
	var menu = document.getElementById(ID);
	if (elem.className != 'click') {
		elem.className = 'click';
		menu.style.display = 'none';
	}
	else {
		elem.className = 'open';
		menu.style.display = 'block';
	}
}