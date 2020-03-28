var make_ins = Dexter.make_ins
var _arcsec = 1/3600
var _nbits_cf = 7754.73550222
var _um = 0.000001
var pidXYZ = 0x3dcCCCCC
//var pidXYZ = 0x3e800000
var pidRP = 0x3dcCCCCC
//var pidBase = 0x3dcCCCCC
var pidBase = pidXYZ
//var DEF_SPEED_FACTOR_A = 30
var DEF_SPEED_FACTOR_DIFF = 10

var recording_state = false
var playing_state = false
var pausing_state = false



Kin.delta_time_to_angle_speed = function(J_angles_original, J_angles_destination, delta_time){
	let delta = Vector.subtract(J_angles_destination, J_angles_original)
    for(let i = 0; i < delta.length; i++){
    	delta[i] = Math.abs(delta[i])
    }
    let max_theta = Vector.max(delta)
    return max_theta/delta_time
}

/*
new Job({
	name: "Loop",
    do_list: [
    	function(){return replayPointsitr(Robot1RecordedPossition,1,0)},
        make_ins("F")//,
        //Robot.go_to(0)
    ]})
*/

var gJobDone = 0
var gbRcdArray = false
var gWindowVals = undefined
var pointIdx = 0
var Robot1Possition = [0, 0, 0, 0, 0, 0]
//var timeXYZSlave = [0, 0, 0, 0, 0, 0]
var bFirstTime = true
var Robot1RecordedPossition = [0, 0, 0, 0, 0, 0]
var RcdpointIdx = 0



//Addresses
var PID_P = 20
var PID_ADDRESS = 21
var DIFF_FORCE_SPEED_FACTOR_ANGLE = 55
var DIFF_FORCE_SPEED_FACTOR_ROT = 56
var SPEED_FACTORA = 27
var DIFF_BETA = 52
var XYZ_BETA = 28





function MysetFollowMe(){
	let pidXYZ  = 0x3e4ecccc //not set in this file
	let pidRP   = 0x3cf5c28f
	let pidBase = 0x3e4ecccc
	let PID_P = 20
	let PID_ADDRESS = 21
	let DIFF_FORCE_SPEED_FACTOR_ANGLE = 55
	let DIFF_FORCE_SPEED_FACTOR_ROT   = 56
	let SPEED_FACTORA = 27
	let DEF_SPEED_FACTOR_A    = 30
	let DEF_SPEED_FACTOR_DIFF = 18
  let DIFF_BETA = 52
  let XYZ_BETA = 28
  console.log("setting Follow mode")

	return [
        make_ins("w", DIFF_BETA, 177),
        make_ins("w", XYZ_BETA, 90),
        make_ins("w", 51, 80000),
        make_ins("w", 54, 60000),

        make_ins("w", DIFF_FORCE_SPEED_FACTOR_ANGLE, DEF_SPEED_FACTOR_DIFF),
        make_ins("w", DIFF_FORCE_SPEED_FACTOR_ROT, DEF_SPEED_FACTOR_DIFF),
        make_ins("w", PID_P, 0),
        make_ins("w", PID_ADDRESS, 3),
        make_ins("w", PID_ADDRESS, 4),
        make_ins("w", PID_ADDRESS, 0),
        make_ins("w", PID_ADDRESS, 1),
        make_ins("w", PID_ADDRESS, 2),
        make_ins("w", SPEED_FACTORA, DEF_SPEED_FACTOR_A),
        make_ins("S", "J1Friction",1) ,
        make_ins("S", "J2Friction",1) ,
        make_ins("S", "J3Friction",1) ,
        make_ins("S", "J4Friction",1),
        make_ins("S", "J5Friction",1) ,
        make_ins("w", 67, 0),
        make_ins("w", 68, 0),
        make_ins("w", 69, 0),
        make_ins("w", 70, 0),
        make_ins("w", 71, 0),

        make_ins("w", 42, 12448)
    ]
}

function MysetFollowMeEasy(){
	var retCMD = []
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ANGLE, DEF_SPEED_FACTOR_DIFF+7))
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ROT, DEF_SPEED_FACTOR_DIFF+7))
    retCMD.push(make_ins("w", DIFF_BETA, 80))

    retCMD.push(make_ins("w", PID_P, 0))
    retCMD.push(make_ins("w", PID_ADDRESS, 3))
    retCMD.push(make_ins("w", PID_ADDRESS, 4))
    retCMD.push(make_ins("w", PID_ADDRESS, 0))
    retCMD.push(make_ins("w", PID_ADDRESS, 1))
    retCMD.push(make_ins("w", PID_ADDRESS, 2))
    retCMD.push(make_ins("w", SPEED_FACTORA, DEF_SPEED_FACTOR_A+1))
    retCMD.push(make_ins("S", "J1Friction",1 ))
    retCMD.push(make_ins("S", "J2Friction",1 ))
    retCMD.push(make_ins("S", "J3Friction",1 ))
    retCMD.push(make_ins("S", "J4Friction",1 ))
    retCMD.push(make_ins("S", "J5Friction",1 ))
    retCMD.push(make_ins("w", 67, 0))
    retCMD.push(make_ins("w", 68, 0))
    retCMD.push(make_ins("w", 69, 0))
    retCMD.push(make_ins("w", 70, 0))
    retCMD.push(make_ins("w", 71, 0))
    //retCMD.push(make_ins("w", 51, 50000))
    //retCMD.push(make_ins("w", 54, 90000))

	//retCMD.push(make_ins("w", 79, 200 ^ 180 ))
    //retCMD.push(make_ins("w", 80, 50 ^ 200 ))
    //retCMD.push(make_ins("w", 81, 50 ^ 200 ))
    retCMD.push(make_ins("w", 42, 12448))
    return retCMD  
}
function MysetForceProtect(){
	var retCMD = []
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ANGLE, 3))
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ROT, 3))
    retCMD.push(make_ins("w", SPEED_FACTORA, 10))
    retCMD.push(make_ins("S", "J1Friction",2 ))
    retCMD.push(make_ins("S", "J2Friction",3 ))
    retCMD.push(make_ins("S", "J3Friction",9 ))
    retCMD.push(make_ins("S", "J4Friction",15 ))
    retCMD.push(make_ins("S", "J5Friction",15 ))
    retCMD.push(make_ins("w", PID_ADDRESS, 0))
    retCMD.push(make_ins("w", PID_P, pidBase))
   	retCMD.push(make_ins("w", PID_ADDRESS, 1))
    retCMD.push(make_ins("w", PID_P, pidXYZ))
  	retCMD.push(make_ins("w", PID_ADDRESS, 2))
  	retCMD.push(make_ins("w", PID_ADDRESS, 3))
  	retCMD.push(make_ins("w", PID_P, pidRP))
  	retCMD.push(make_ins("w", PID_ADDRESS, 4))
    retCMD.push(make_ins("w", 67, 9000))
    retCMD.push(make_ins("w", 68, 9000))
    retCMD.push(make_ins("w", 69, 9000))
    retCMD.push(make_ins("w", 70, 9000))
    retCMD.push(make_ins("w", 71, 9000))
    retCMD.push(make_ins("w", 42, 12448))
    return retCMD  
}

function MysetKeepPossition(){
	var retCMD = []
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ANGLE, 0))
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ROT, 0))
    retCMD.push(make_ins("w", PID_ADDRESS, 0))
    retCMD.push(make_ins("w", PID_P, pidBase))
   	retCMD.push(make_ins("w", PID_ADDRESS, 1))
    retCMD.push(make_ins("w", PID_P, pidXYZ))
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ROT, 0))
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ROT, 0))
  	retCMD.push(make_ins("w", PID_ADDRESS, 2))
  	retCMD.push(make_ins("w", PID_ADDRESS, 3))
  	retCMD.push(make_ins("w", PID_P, pidRP))
  	retCMD.push(make_ins("w", PID_ADDRESS, 4))
  	retCMD.push(make_ins("w", SPEED_FACTORA, 0))
    retCMD.push(make_ins("w", 42, 12960))
    return retCMD
}

let OldStartTime = 0
let OldEndTime = 0

function updateXYZPoint(){
    	var xyzPoint = [Job.j1.robot.robot_status[Dexter.J1_MEASURED_ANGLE], 
						Job.j1.robot.robot_status[Dexter.J2_MEASURED_ANGLE], 
                        Job.j1.robot.robot_status[Dexter.J3_MEASURED_ANGLE],
                        Job.j1.robot.robot_status[Dexter.J4_MEASURED_ANGLE], 
                        Job.j1.robot.robot_status[Dexter.J5_MEASURED_ANGLE],
                        Job.j1.robot.robot_status[Dexter.J6_MEASURED_ANGLE], Job.j1.robot.robot_status[Dexter.J7_MEASURED_ANGLE],
                        -(OldStartTime - Job.j1.robot.robot_status[Dexter.START_TIME] + ((OldEndTime - Job.j1.robot.robot_status[Dexter.STOP_TIME]) / 1000000))]
        
        //console.log(-(OldStartTime - Dexter.my_dex.robot_status[Dexter.START_TIME] + ((OldEndTime - Dexter.my_dex.robot_status[Dexter.STOP_TIME]) / 1000000)))
        OldStartTime = Job.j1.robot.robot_status[Dexter.START_TIME]
        OldEndTime = Job.j1.robot.robot_status[Dexter.STOP_TIME]
        

return xyzPoint
}

/*function updateXYZPointSlave(){
    	var xyzPoint = [Dexter.my_dex2.robot_status[Dexter.J1_FORCE_CALC_ANGLE] + Dexter.my_dex2.robot_status[Dexter.J1_DELTA], 
						Dexter.my_dex2.robot_status[Dexter.J2_FORCE_CALC_ANGLE] + Dexter.my_dex2.robot_status[Dexter.J2_DELTA], 
                        Dexter.my_dex2.robot_status[Dexter.J3_FORCE_CALC_ANGLE] + Dexter.my_dex2.robot_status[Dexter.J3_DELTA],
                        Dexter.my_dex2.robot_status[Dexter.J4_FORCE_CALC_ANGLE] + (Dexter.my_dex2.robot_status[Dexter.J4_DELTA] / 16), 
                        Dexter.my_dex2.robot_status[Dexter.J5_FORCE_CALC_ANGLE] + (Dexter.my_dex2.robot_status[Dexter.J5_DELTA] / 16)]
        return xyzPoint
}
*/
var StopPlayBack = false
var SetSlavePlayback = false
function handleWindowUI(vals){ 
  gWindowVals = vals 
  //console.log(vals)
  if(vals.clicked_button_value == "SetPoint" ){ // Clicked button value holds the name of the clicked button.
    StopPlayBack = true
    //if(gbRcdArray == false){gbRcdArray = true}else{gbRcdArray = false}
    //console.log("Recording: " + gbRcdArray)

    //Robot1Possition[pointIdx] = updateXYZPoint()
  }
  else if(vals.clicked_button_value == "Follow" ) { 
    Job.j1.user_data.choicemade = function (){return MysetFollowMeEasy()}
    console.log("Set FollowMe mode")
  }
  else if(vals.clicked_button_value == "Keep" ){ 
    Job.j1.user_data.choicemade = function (){return MysetKeepPossition()}
    console.log("Set set Keep  mode")

  }
  else if(vals.clicked_button_value == "Protect" ){ 
    Job.j1.user_data.choicemade = function (){return MysetForceProtect()}
  }
  else if(vals.clicked_button_value == "Play" ){ 
    out ("playBack")
    Job.j1.user_data.choicemade = function ()
    { 
      SetSlavePlayback = true
      var rt = []
      rt.push(function(){Job.j1.inter_do_item_dur = .002})
      rt.push(make_ins("S", "ServoSet2X", 1, 24, 257)) 
      rt.push(make_ins("S", "ServoSet2X", 3, 24, 257)) 
      //rt.push(make_ins("S", "ServoSet", 3, 24, 1)) // set torque to max
      //rt.push(make_ins("S", "ServoSet", 1, 24, 1)) // set torque to max

      rt.push(MysetKeepPossition())
      rt.push(Robot.wait_until(1))
      rt.push(make_ins("w", 36,2000 ^ 18000))
      //rt.push(moveBoundry([0,0,0,0,0]))
      rt.push(Robot.wait_until(1))
      //rt.push(MysetFollowMe())
      rt.push(Robot.wait_until(1))
      rt.push(function(){return replayPointsitr(Robot1RecordedPossition,10,0)})
      rt.push(function(){Job.j1.inter_do_item_dur = .005})
      rt.push(make_ins("S", "ServoSet2X", 1, 24, 512)) // set torque to min
      rt.push(make_ins("S", "ServoSet2X", 3, 24, 512)) // set torque to min
      rt.push(make_ins("w", 36,2000 ^ 6000))

      //rt.push(make_ins("S", "ServoSet", 3, 24, 0)) // set torque to off
      //rt.push(make_ins("S", "ServoSet", 1, 24, 0)) // set torque to off
      return rt                                        
    }
  }
  else if(vals.clicked_button_value == "Read" ){ 
  	let file_path = choose_file()
    Robot1RecordedPossition = load_files(file_path)
    
    //Robot1RecordedPossition = load_files(vals.macro_name)
  }
  else if(vals.clicked_button_value == "Write" ){ 
  	let file_path = choose_save_file()
    write_file(file_path, JSON.stringify(Robot1RecordedPossition))
    //write_file(vals.macro_name, JSON.stringify(Robot1RecordedPossition))

  }

  else if (vals.clicked_button_value == "Done" ){   
    gJobDone = 1
    gWindowVals = undefined
    console.log("outta here " )
  }
}

let ReplaySpeed = 20 
//var [].shift
var PointsHistory = [[0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0]]
var MeasuredHistorySlope = [0, 0, 0, 0, 0]
//Vector.subtract(CommandHistorySlope, MeasuredHistorySlope)


function CheckCollission()
{  
  let CurrentAngle = updateXYZPoint()
  let CartPoint = Kin.J_angles_to_xyz(CurrentAngle.slice(0, 5))[0]
  let DeltaTime = CurrentAngle[7]
  //CartPoint.push(DeltaTime)
  PointsHistory.push(CartPoint)
  let dummy = PointsHistory.shift()
  let MeasAngleDelta = Vector.subtract(PointsHistory[4], PointsHistory[3]) 
  let MeasAngleDeltaMag = Vector.magnitude(MeasAngleDelta) / DeltaTime
//  let MeasAngleDeltaMag = Vector.subtract(Vector.magnitude(PointsHistory[4]), 
//                                          Vector.magnitude(PointsHistory[3])) 
  let PointsHistoryAverage = Vector.average(MeasuredHistorySlope)

  MeasuredHistorySlope.push( MeasAngleDeltaMag)
  dummy = MeasuredHistorySlope.shift()
  let secondD = MeasuredHistorySlope[4] - PointsHistoryAverage
  if(Math.abs(secondD) > .2)
  {
    out(secondD)
    return Robot.break()
  }
}

function replayPointsitr(points, times,gripOffset ){
 let PCurrentAngle = updateXYZPoint().slice(0, 5)
 let PCartPoint = Kin.J_angles_to_xyz(PCurrentAngle)[0]

 PointsHistory = [PCartPoint, PCartPoint, PCartPoint, PCartPoint, PCartPoint]
 MeasuredHistorySlope = [0, 0, 0, 0, 0]

 var rt =[]
 let SyncNum = 0
 let SyncInterval = 10
 let SyncCount = 0
 for (let j=0;j<times;j++)
 {
   for (let i = 0;i < points.length;i=i+1)
   {
     rt.push(function (){ return CheckCollission()})
     if(SyncCount++ > SyncInterval){
       SyncCount = 0
       //rt.push(Robot.sync_point("sync" + SyncNum++, ["j1", "j2"]))
     }
     rt.push(Dexter.pid_move_all_joints([points[i][0], points[i][1] , points[i][2] , points[i][3] , points[i][4], points[i][5], points[i][6] ]))
   }
   
   let LastPoint = points[points.length-1]
   let DeltaP = Vector.subtract([0,0,0,0,0,0,0,0], LastPoint)
   let ReturnSize = Vector.max(Vector.abs(Vector.divide(DeltaP.slice(0,5), .3)))  // 

   let DeltaDiff = Vector.divide(DeltaP, ReturnSize)
   for (var i = 0;i < ReturnSize;i=i+1){
     let NewPoint = Vector.add(LastPoint,Vector.multiply(DeltaDiff,i))
     rt.push(Dexter.pid_move_all_joints([NewPoint[0], NewPoint[1] , NewPoint[2] , NewPoint[3] , NewPoint[4] ]))
   }
   rt.push(Dexter.pid_move_all_joints([0, 0, 0, 0, 0 ]))
   rt.push(function (){return WaitForHome()})

 }
 return rt
}
/*
show_window({content:
`<input name="SetPoint" type="button" accesskey="p" value="SetPoint"/> 
 <input name="Follow" type="button" value="Follow"/>
 <input name="Keep" type="button" value="Keep"/>
 <input name="Protect" type="button" value="Protect"/>
 <input name="Play" type="button" value="Play"/>  <br/><br/>
  <input type="button" value="Read"/>
 <input type="button" value="Write"/>

  Mode: <span id="mode_id">None</mode><br/>
 X: <span  name="X_display" id="X_id">0</span><br/>
 Y: <span  name="Y_display" id="Y_id">0</span><br/>
 Z: <span  name="Z_display" id="Z_id">0</span><br/><br/>
 Macro Name <input type="text" name="macro_name" id="mn_id" <br/><br/>
 <input type="submit" value="Done"/>`, 
             callback: handleWindowUI})     
*/




function resolve_choice()
{
//debugger;
	var na = []
    na.push(make_ins("g"))
    na.push(function(){if (this.user_data.choicemade != undefined) {
    		var rtval = this.user_data.choicemade
            this.user_data.choicemade = undefined
            return rtval
    		}})
   
    na.push(function(){if (gbRcdArray == true) {Robot1Possition[pointIdx] = updateXYZPoint()
          //console.log(" raw robot pos  " + Robot1Possition[pointIdx])
          Robot1RecordedPossition[RcdpointIdx++] = Robot1Possition[pointIdx]
          var rtval = []
          return rtval

    }})	
    na.push(function(){if (gJobDone == 0) {return resolve_choice()}})
    return na
}

function moveBoundry(angles)
{
  let window = 0
  let rt = []
  rt.push(make_ins("B",  Math.round(-angles[0]+window), Math.round(-angles[0]-window), Math.round(-angles[1]+window), Math.round(-angles[1]-window), 
  Math.round(-angles[2]+window), Math.round(-angles[2]-window), Math.round(-angles[3]+window), Math.round(-angles[3]-window), Math.round(angles[4]+window), Math.round(angles[4]-window)))
  return rt

}
function setBoundary(){
let rt = []
  rt.push(make_ins("S", "J1BoundryHigh",610000*_arcsec))
  rt.push(make_ins("S", "J1BoundryLow",-610000*_arcsec))
  rt.push(make_ins("S", "J2BoundryLow",-310000*_arcsec))
  rt.push(make_ins("S", "J2BoundryHigh",320000*_arcsec))
  rt.push(make_ins("S", "J3BoundryLow",-500000*_arcsec))
  rt.push(make_ins("S", "J3BoundryHigh",500000*_arcsec))
  rt.push(make_ins("S", "J4BoundryLow",-290000*_arcsec))
  rt.push(make_ins("S", "J4BoundryHigh",290000*_arcsec))
  rt.push(make_ins("S", "J5BoundryLow",-610000*_arcsec))
  rt.push(make_ins("S", "J5BoundryHigh",610000*_arcsec))
return rt
}
function SetPHUIMode()
{
	let pidXYZ  = 0x3e4ecccc //not set in this file
	let pidRP   = 0x3e4ecccc
	let pidBase = 0x3e4ecccc
	let PID_P = 20
	let PID_ADDRESS = 21
	let DIFF_FORCE_SPEED_FACTOR_ANGLE = 55
	let DIFF_FORCE_SPEED_FACTOR_ROT   = 56
	let SPEED_FACTORA = 27
	let DEF_SPEED_FACTOR_A    = 52
	let DEF_SPEED_FACTOR_DIFF = 17
    let DIFF_BETA = 52
	let XYZ_BETA = 28

	return [
        make_ins("w", 51, 40000),
        make_ins("w", 54, 40000),


    	Dexter.set_parameter("J5Force", 0),
    	Dexter.set_parameter("J4Force", 0),
        make_ins("w", DIFF_BETA, 160),
        make_ins("w", XYZ_BETA, 20),

        make_ins("w", DIFF_FORCE_SPEED_FACTOR_ANGLE, DEF_SPEED_FACTOR_DIFF),
        make_ins("w", DIFF_FORCE_SPEED_FACTOR_ROT, DEF_SPEED_FACTOR_DIFF),
        make_ins("w", PID_ADDRESS, 4),
        make_ins("w", PID_P, 0),
        make_ins("w", PID_ADDRESS, 3),
        //make_ins("w", PID_ADDRESS, 4),
        //make_ins("w", PID_ADDRESS, 0),
        //make_ins("w", PID_ADDRESS, 1),
        //make_ins("w", PID_ADDRESS, 2),
        //make_ins("w", SPEED_FACTORA, DEF_SPEED_FACTOR_A),
        make_ins("S", "J1Friction",1) ,
        make_ins("S", "J2Friction",1) ,
        make_ins("S", "J3Friction",1) ,
        make_ins("S", "J4Friction",27),
        make_ins("S", "J5Friction",15) ,
        make_ins("w", 67, 0),
        make_ins("w", 68, 0),
        make_ins("w", 69, 0),
        make_ins("w", 70, 0),
        make_ins("w", 71, 0),

        make_ins("w", 42, 12448)
    ]
}

function J5Interface(MyJob)
{
  let TickDeg = 10
  let rs = MyJob.robot.robot_status[Dexter.J5_MEASURED_ANGLE]
  let quant = Math.round(rs/TickDeg)
  let fract = (quant * TickDeg) - rs
  if(quant != MyJob.user_data.LastUISelectRotate)
  {
    MyJob.user_data.LastUISelectRotate = quant
    out(MyJob.user_data.LastUISelectAngle + "  " + MyJob.user_data.LastUISelectRotate)
  }
  
  let BackPressure =  - ((fract) * 100)
  return (Dexter.set_parameter("J5Force", BackPressure))
}
function J4Interface(MyJob)
{
  let TickDeg = 15
  let rs = MyJob.robot.robot_status[Dexter.J4_MEASURED_ANGLE]
  let quant = Math.round(rs/TickDeg)
  let fract = (quant * TickDeg) - rs
  if(quant != MyJob.user_data.LastUISelectAngle)
  {
    MyJob.user_data.LastUISelectAngle = quant
    out(MyJob.user_data.LastUISelectAngle + "  " + MyJob.user_data.LastUISelectRotate)
	out("ticks " + TickDeg)
  }

  let BackPressure =  - (fract * 100)
  return (Dexter.set_parameter("J4Force", BackPressure))
}

function PHUIcalc(MyJob)
{
  let rt = []
  //debugger;
  rt.push(SetPHUIMode())
  rt.push(Robot.loop(true, function(MyJob)
  {
    let CMD = []
    CMD.push(Dexter.get_robot_status())
    CMD.push(function(MyJob)
    {  
       let rt = []
       rt.push(J4Interface(Job.j1))
       rt.push(J5Interface(Job.j1))
       if(Job.j1.user_data.LastUISelectAngle == 2  || Job.j1.user_data.LastUISelectAngle == -2){
          return [Dexter.set_parameter("J5Force", 0),
          Dexter.set_parameter("J4Force", 0),
          Robot.break]}
       else {return rt}
       
    })
    return CMD
  }
  ))
  rt.push(MysetKeepPossition())
  //rt.push(WaitForHome())
  return rt
}

function WaitForHome()
{
  return Robot.loop(true, function()
                    {
    let rt = []
    rt.push(Dexter.get_robot_status())
    rt.push(function ()
            {   
             let MeasAng = updateXYZPoint().slice(0,5)
             let AngDelta = Vector.subtract(MeasAng, [0, 0, 0, 0, 0])
             let MaxAngle = Vector.max(Vector.abs(AngDelta))


             if(MaxAngle < .1){return Robot.break()}
            })
    return rt
  })

}
var FileNotFound = false

new Job({name: "j1", robot: Robot.dexter0, keep_history: false, show_instructions: false, user_data: {LastUISelectRotate: 0, LastUISelectAngle: 0},
         do_list: [	
            Dexter.move_all_joints(0,0,0,10,0),
            Dexter.move_all_joints(0,0,0,10,10),
            Dexter.move_all_joints(0,0,0,-10,10),
            Dexter.move_all_joints(0,0,0,-10,-10),
            Dexter.move_all_joints(0,0,0,0,0),

           
           function ()
           {
             Robot1RecordedPossition = []
             RcdpointIdx = 0
             FileNotFound = false
           },
           make_ins("S", "RebootServo", 1),
           make_ins("S", "RebootServo", 3),
           make_ins("S", "ServoSet2X", 1, 15, 1023),
           make_ins("S", "ServoSet2X", 3, 15, 1023), // max torque
           make_ins("S", "ServoSet2X", 1, 35, 1023),
           make_ins("S", "ServoSet2X", 3, 35, 1023), // max torque
           //Dexter.move_all_joints(10, 0, 0, 0, 0),
           make_ins("S", "ServoSet2X", 1, 24, 512), 
           make_ins("S", "ServoSet2X", 3, 24, 512), 
           Dexter.pid_move_all_joints(0,0,0,0,0),

           MysetKeepPossition(),
           //moveBoundry([0,0,0,0,0]),


           //make_ins("w", 36,2000 ^ 3000),  // adjust max error

           make_ins("S", "MaxSpeed",40),
           make_ins("S", "StartSpeed",1),
           make_ins("S", "Acceleration",.0001),
           make_ins("w", 36,2000 ^ 3000),
           function(){Job.j1.inter_do_item_dur = .000001},
           PHUIcalc(this),
           function(){Job.j1.inter_do_item_dur = .005},
           MysetKeepPossition(),
	   function (){return WaitForHome()},
            function() 
            {
              if(this.user_data.LastUISelectRotate == 0 && this.user_data.LastUISelectAngle == - 2){return Robot.stop_job()}
            },
           function ()
           {   
               if(this.user_data.LastUISelectRotate !== 0 && this.user_data.LastUISelectAngle == - 2)
               {
                  out("Load In program " + this.user_data.LastUISelectRotate)
                  let file_path = "PreSet" + (parseInt(this.user_data.LastUISelectRotate, 10) + 16)
                  try{
                  let contents = file_content(file_path)
                  }
                  catch(error) {FileNotFound = true}
                  if(FileNotFound == false)
                  {
                    Robot1RecordedPossition=JSON.parse(file_content(file_path))
                  }
                  
               }
               else
               {
                 return [           
                 	function()
                    {
                      gbRcdArray = true
           			  Robot1Possition[pointIdx] = updateXYZPoint()
           			  out("Recording: " + gbRcdArray)
                    },
           			Dexter.move_all_joints(0,2,2,0,0),
           			Dexter.move_all_joints(0,0,0,0,0),
           			make_ins("F"),
           			Dexter.sleep(1),
                    MysetFollowMe(),           
                    Robot.loop(true, function()
                    {
                      let CMD = []
                      CMD.push(Dexter.get_robot_status())
                      CMD.push(function(){Robot1Possition[pointIdx] = updateXYZPoint()
                      Robot1RecordedPossition[RcdpointIdx++] = Robot1Possition[pointIdx]})
                      CMD.push(function()
                      {
                        let thres = -20.4
                        let rs = Job.j1.robot.robot_status
                        if(rs[Dexter.J2_FORCE_CALC_ANGLE] < thres){
                        return Robot.break}
                      })
                      return CMD
                    }),
                    function ()
                    {
                      out("stop recording")
                      gbRcdArray = false
                    },
                    MysetKeepPossition(),
			        function (){return WaitForHome()},
                    Dexter.sleep(.1)]
               }
           },
 
           function ()
           {   
               if(this.user_data.LastUISelectRotate !== 0 && this.user_data.LastUISelectAngle == 2)
               {
                  out("Save This As " + this.user_data.LastUISelectRotate)
                  let file_path = "PreSet" + (parseInt(this.user_data.LastUISelectRotate, 10) + 16)
     			  write_file(file_path, JSON.stringify(Robot1RecordedPossition))

               }
           },
           Robot.loop(true, function(){ 
             if(FileNotFound == false)
             {
               let rt = []
               rt.push(function(){Job.j1.inter_do_item_dur = .005})
               rt.push(make_ins("S", "ServoSet2X", 1, 24, 257)) 
               rt.push(make_ins("S", "ServoSet2X", 3, 24, 257)) 
               //rt.push(make_ins("S", "ServoSet", 3, 24, 1)) // set torque to max
               //rt.push(make_ins("S", "ServoSet", 1, 24, 1)) // set torque to max

               rt.push(MysetKeepPossition())
               rt.push(Robot.wait_until(.1))
               rt.push(make_ins("w", 36,2000 ^ 18000))
               //rt.push(moveBoundry([0,0,0,0,0]))
               rt.push(Robot.wait_until(.1))
               //rt.push(MysetFollowMe())
               rt.push(Robot.wait_until(.1))
               rt.push(function(){return replayPointsitr(Robot1RecordedPossition,1,0)})
               rt.push(function(){Job.j1.inter_do_item_dur = .005})
               rt.push(make_ins("S", "ServoSet2X", 1, 24, 512)) // set torque to min
               rt.push(make_ins("S", "ServoSet2X", 3, 24, 512)) // set torque to min


               //rt.push(make_ins("S", "ServoSet", 3, 24, 0)) // set torque to off
               //rt.push(make_ins("S", "ServoSet", 1, 24, 0)) // set torque to off
               rt.push(function(){
                 if(StopPlayBack == true){
                   return Robot.break}
               })

               return rt
             }
             else {return Robot.break}
           }),
           function ()
           {     
             let rt = []
             let PCurrentAngle = updateXYZPoint().slice(0, 5)
             rt.push(Dexter.pid_move_all_joints([PCurrentAngle[0], PCurrentAngle[1] , PCurrentAngle[2] , PCurrentAngle[3] , PCurrentAngle[4] ]))
             return rt

           },



           // function(){return resolve_choice},
           make_ins("w", 36,2000 ^ 6000),
           function (){StopPlayBack = false},
           MysetKeepPossition(),
           //moveBoundry([0,0,0,0,0]),
           Dexter.pid_move_all_joints(0,0,0,0,0),
           Dexter.move_all_joints(0, 0, 0, 0, 0),
           Robot.wait_until(3),
           Robot.go_to(0)
         ]})


