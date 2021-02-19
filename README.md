<H1> MK3 swerve calibration instructions</h1>
<b>Please read through entire readme before deploying to robot.</b>
<ol>
  <li>Make sure your firmware is relitivly up to date!</li>
      <ol>
      <li> Our bots current firmware versions:</li>
      <li> Roborio: 2021_v3</li>
      <li> Falcon500's: 21.0.0</li>
      <li> CANCoder's: 20.1 </li>
      </ol>
  <li>While you are in the phoenix tuner make sure you know what each motor and cancoders CAN ids are.</li>
  <li>Clone this repository!</li>
  <li>Enter the CAN id's into lines 34-49 in SwerveDrivetrain.java. It is at this point you will choose a "front". This is important for later.</li>
  <li><b>Comment out line 109 in SwerveDrivetrain.java in addtion to lines 96 and 101 in SwerveModuleMK3.java</b></li>
  <li>Deploy to robot and enable.</li>
  <li>turn each wheel to point to the "front" you have choosen.</li>
  <li>Open the smartdashboard and record the numbers in the boxes numbered 0-3.</li>
  <li> Enter the numbers gathered above into their corresponding offset in SwerveDrivetrain.java lines 29-32.
      <ol>
      <li> 0: front left</li>
      <li> 1: front right</li>
      <li> 2: back left</li>
      <li> 3: back right</li>
      </ol>
   <li>Uncomment line 109 in SwerveDrivetrain.java in addtion to lines 96 and 101 in SwerveModuleMK3.java</li>
  <li> Happy swerving!</li>
</ol>
