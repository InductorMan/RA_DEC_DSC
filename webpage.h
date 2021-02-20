const char webpage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<style type="text/css">
.button {
  background-color: #4CAF50; /* Green */
  border: none;
  color: white;
  padding: 15px 32px;
  text-align: center;
  text-decoration: none;
  display: inline-block;
  font-size: 16px;
}
</style>
<body style="background-color: #000000;color:#800000 ">
<center>
<div>
<h1>InductorMan's Digital Setting Circles</h1>
  <h2>
    RA: <span id="right_ascention">0</span><br><br>
    HA: <span id="hour_angle">0</span><br><br>
    Dec: <span id="declination">NA</span>
  </h2>
</div>
 <br>
<div>
  <form action="/set_ra">
    <label for="ra_hr">RA Set: </label>
    <input style="background-color: #300000;color:#A00000 " type="text" id="ra_hr" name="ra_hr">
    <label for="ra_hr">hr </label>
    <input style="background-color: #300000;color:#A00000 " type="text" id="ra_min" name="ra_min">
    <label for="ra_min">min </label>
    <input style="background-color: #300000;color:#A00000 " type="text" id="ra_sec" name="ra_sec">
    <label for="ra_sec">sec </label>
    <input style="background-color: #300000;color:#800000 " type="submit" value="Set">
  </form><br>
  <form action="/set_dec">
    <label for="dec_deg">Dec Set: </label>
    <input style="background-color: #300000;color:#A00000 " type="text" id="dec_deg" name="dec_deg">
    <label for="dec_deg">hr </label>
    <input style="background-color: #300000;color:#A00000 " type="text" id="dec_min" name="dec_min">
    <label for="dec_min">min </label>
    <input style="background-color: #300000;color:#800000 " type="submit" value="Set">
  </form><br>
</div>
<script>
setInterval(function() 
{
  getData();
}, 500); 
function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var data = this.responseText.split("\n");
      document.getElementById("right_ascention").innerHTML =
      data[0];
      document.getElementById("hour_angle").innerHTML =
      data[1];
      document.getElementById("declination").innerHTML =
      data[2];
    }
  };
  xhttp.open("GET", "pos_read", true);
  xhttp.send();
}
</script>
</center>
</body>
</html>
)=====";
