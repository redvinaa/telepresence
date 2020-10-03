const ROT_FREQ = 10

var el;

window.onload = async function () {

	el      = await document.getElementById('screen');


	var data  = await $.get('/get_data').promise();
	data = data.split('#');
	var websocket_address  = data[0];
	var move_base_instance = data[1];
	var cmd_vel_topic      = data[2];
	var stream_url         = data[3];
	var ROT_VEL            = parseFloat(data[4]);


	if (!el) {throw "Could not get canvas element"};
	el.setAttribute("src", stream_url);


	ros = new ROSLIB.Ros({
		url: websocket_address
	});

	ros.on('connection', function() {
		console.log('Connected to websocket server');
	});

	ros.on('error', function(error) {
		console.log('Error connecting to websocket server: ', error);
		alert('Error connecting to websocket server: ', error);
	});

	ros.on('close', function() {
		console.log('Connection to websocket server closed.');
	});


	var srvClick = new ROSLIB.Service({
		ros: ros,
		name: "/telepresence/click",
		serviceType: "telepresence/Click"
	});




	var pubRotate = new ROSLIB.Topic({
		ros: ros,
		name: cmd_vel_topic,
		messageType: "geometry_msgs/Twist"
	});
	var rotateMessage = new ROSLIB.Message({});

	var pubCancelGoal = new ROSLIB.Topic({
		ros: ros,
		name: '/'+move_base_instance+'/cancel',
		messageType: 'actionlib_msgs/GoalID'
	});
	var cancelMessage = new ROSLIB.Message({});

	$("#cancel").on('click', function(event) {
		pubCancelGoal.publish(cancelMessage);
	});


	var btn_left       = document.getElementById("left");
	var btn_right      = document.getElementById("right");
	var btn_cancel     = document.getElementById("cancel");
	var btn_color_orig = btn_left.style.background;
	var btn_color_canc = btn_cancel.style.background;
	var btn_color_down = "white"

	var btnInterval;

	btn_left.onmousedown = () => {

		rotateMessage = new ROSLIB.Message({
			linear: {x: 0, y: 0, z: 0},  angular: {x: 0, y: 0, z: ROT_VEL}
		})
		btnInterval = window.setInterval(()=>{
			console.log('rotating left...')
			pubRotate.publish(rotateMessage);
		}, 1000/ROT_FREQ);
		btn_left.style.background = btn_color_down;
	}

	btn_left.onmouseup = () => {
		window.clearInterval(btnInterval);
		rotateMessage.angular.z = 0;
		pubRotate.publish(rotateMessage);
		btn_left.style.background = btn_color_orig;
	}

	btn_right.onmousedown = () => {

		rotateMessage = new ROSLIB.Message({
			linear: {x: 0, y: 0, z: 0},  angular: {x: 0, y: 0, z: -ROT_VEL}
		})
		btnInterval = window.setInterval(()=>{
			console.log('rotating right...')
			pubRotate.publish(rotateMessage);
		}, 1000/ROT_FREQ);
		btn_right.style.background = btn_color_down;
	}

	btn_right.onmouseup = () => {
		window.clearInterval(btnInterval);
		rotateMessage.angular.z = 0;
		pubRotate.publish(rotateMessage);
		btn_right.style.background = btn_color_orig;
	}

	btn_cancel.onmousedown = () => {
		btn_cancel.style.background = btn_color_down;
	}

	btn_cancel.onmouseup = () => {
		btn_cancel.style.background = btn_color_canc;
	}



	el.onclick = function(event) {
		var x = (event.pageX - el.offsetLeft) / el.width;
		var y = (event.pageY - el.offsetTop) / el.height;

		var request = new ROSLIB.ServiceRequest({
			x: x,
			y: y
		});

		console.log("Sending request...");

		var inv_pos = document.getElementById("invalid_pos");

		srvClick.callService(request, function(result) {
			console.log("Calling service " + srvClick.name + ", success: " + result.success);
			if (!result.success) {
				alert(result.message);
			}
		});
	}
}
