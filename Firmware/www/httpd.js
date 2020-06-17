
var http = require('http'); 
var url = require('url'); //url parsing
var fs = require('fs'); //file system
var net = require('net'); //network
const ws = require('ws'); //websocket
// https://github.com/websockets/ws 
//install with:
//npm install --save ws 
//on Dexter, if httpd.js is going to be in the /srv/samba/share/ folder, 
//install ws there but then run it from root. e.g. 
//cd /srv/samba/share/ 
//npm install --save ws 
//cd /
//node /srv/samba/share/httpd.js 

//var mime = require('mime'); //translate extensions into mime types
//skip that,it's stupidly big
var mimeTypes = {
  "css": "text/css",
  "html": "text/html",
  "gif": "image/gif",
  "jpeg": "image/jpeg",
  "jpg": "image/jpeg",
  "js": "text/javascript",
  "mp3": "audio/mpeg",
  "mp4": "video/mp4",
  "png": "image/png",
  "svg": "image/svg+xml",
  "txt": "text/plain"
  };

const { spawn } = require('child_process');
 
var job_name_to_process = {}
function get_job_name_to_process(job_name) { 
     if(job_name_to_process.keep_alive) { //if there is such a process, then keep_alive is true
     	return job_name_to_process.keep_alive
     }
     else {
        return job_name_to_process[job_name]
     }
}
function set_job_name_to_process(job_name, process) { job_name_to_process[job_name] = process }
function remove_job_name_to_process(job_name) { delete job_name_to_process[job_name] }       
        
//arg looks like "myjob.js", "myjob.dde", "myjob"
function extract_job_name(job_name_with_extension){
	let dot_pos = job_name_with_extension.indexOf(".")
    let job_name
    if(dot_pos === -1) { job_name = job_name_with_extension }
    else { job_name = job_name_with_extension.substring(0, dot_pos) }
    return job_name
}

function serve_init_jobs(q, req, res){
    //console.log("top of serve_init_jobs in server")
    fs.readdir("/srv/samba/share/dde_apps/", 
        function(err, items){
            let items_str = JSON.stringify(items)
            //console.log("serve_init_jobs writing: " + items_str)
            res.write(items_str)
            res.end()
        })
}

//https://www.npmjs.com/package/ws
console.log("now making wss")
const wss = new ws.Server({port: 3001})    //server: http_server });
console.log("done making wss: " + wss)
 

function serve_job_button_click(browser_socket, mess_obj){
    let job_name_with_extension = mess_obj.job_name_with_extension //includes ".js" suffix 
    console.log("\n\nserver top of serve_job_button_click with job_name_with_extension: " + job_name_with_extension)
	let jobfile = "/srv/samba/share/dde_apps/" + job_name_with_extension //q.search.substr(1)
    //console.log("serve_job_button_click with jobfile: " + jobfile)
    let job_name = extract_job_name(job_name_with_extension) 
    //console.log("top of serve_job_button_click with job_name: " + job_name)
    //console.log("serve_job_button_click with existing status_code: " + status_code)
   let job_process = get_job_name_to_process(job_name) //warning: might be undefined.
   //let server_response = res //to help close over
   if(!job_process){
        //https://nodejs.org/api/child_process.html
        //https://blog.cloudboost.io/node-js-child-process-spawn-178eaaf8e1f9
        job_process = spawn('node',
                            ["core define_and_start_job " + jobfile],   //a jobfile than ends in "/keep_alive" is handled specially in core/index.js
                            {cwd: '/root/Documents/dde', shell: true}
                           )
        set_job_name_to_process(job_name, job_process)
        console.log("just set job_name: " + job_name + " to new process: " + job_process)
        job_process.stdout.on('data', function(data) {
          console.log("\n\nserver: stdout.on data got data: " + data + "\n")
          let data_str = data.toString()
          //server_response.write(data_str) //pipe straight through to calling browser's handle_stdout
          //https://github.com/expressjs/compression/issues/56 sez call flush even though it isn't documented.
          //server_response.flushHeaders() //flush is deprecated.
          browser_socket.send(data_str)
	     })
         
        job_process.stderr.on('data', function(data) {
          console.log("\n\njob: " + job_name + " got stderr with data: " + data)
          remove_job_name_to_process(job_name)
          //server_response.write("Job." + job_name + " errored with: " + data)
          let lit_obj = {job_name: job_name,
                         kind: "show_job_button",
                         button_tooltip: "Server errored with: " + data, 
                         button_color: "red"}
          browser_socket.send("<for_server>" + JSON.stringify(lit_obj) + "</for_server>")
          //server_response.end()
          })
        job_process.on('close', function(code) {
          console.log("\n\nServer closed the process of Job: " + job_name + " with code: " + code)
          if(code !== 0){
          	let lit_obj = {job_name: job_name, 
                           kind: "show_job_button",
                           button_tooltip: "Errored with server close error code: " + code,
                           button_color: "red"}
          	browser_socket.send("<for_server>" + JSON.stringify(lit_obj) + "</for_server>")
          }
          remove_job_name_to_process(job_name)
          //server_response.end()
          })
        }
    else {
    	let code 
        if(job_name === "keep_alive") { //happens when transition from keep_alive box checked to unchecked
        	code = "set_keep_alive_value(" + mess_obj.keep_alive_value + ")"
        }
        else {
        	//code = "Job." + job_name + ".server_job_button_click()"
            code = 'Job.maybe_define_and_server_job_button_click("' + jobfile + '")'
        }
        console.log("serve_job_button_click writing to job: " + job_name + " stdin: " + code)
        //https://stackoverflow.com/questions/13230370/nodejs-child-process-write-to-stdin-from-an-already-initialised-process
        job_process.stdin.setEncoding('utf-8');
        job_process.stdin.write(code + "\n")
        //job_process.stdin.end(); 
    }
    //serve_get_refresh(q, req, res)
    //return serve_jobs(q, req, res)  //res.end()
}

//see bottom of je_and_browser_code.js: submit_window for
//the properties of mess_obj
function serve_show_window_call_callback(browser_socket, mess_obj){
    let callback_arg = mess_obj.callback_arg
    let job_name = callback_arg.job_name
    let job_process = get_job_name_to_process(job_name)
    console.log("\n\nserve_show_window_call_callback got job_name: " + job_name + " and its process: " + job_process)
    let code = mess_obj.callback_fn_name + "(" +
               JSON.stringify(callback_arg) + ")"
    //code = mess_obj.callback_fn_name + '({"is_submit": false})' //out('short str')" //just for testing
    console.log("serve_show_window_call_callback made code: " + code)
    job_process.stdin.setEncoding('utf-8');
    job_process.stdin.write(code + "\n") //need the newline so that stdio.js readline will be called
}

function serve_file(q, req, res){
	var filename = "/srv/samba/share/www/" + q.pathname
    console.log("serving" + q.pathname)
    fs.readFile(filename, function(err, data) {
        if (err) {
            res.writeHead(404, {'Content-Type': 'text/html'})
            return res.end("404 Not Found")
        }  
        res.setHeader('Access-Control-Allow-Origin', '*');
        let mimeType = mimeTypes[ q.pathname.split(".").pop() ] || "application/octet-stream"
        console.log("Content-Type:", mimeType)
        res.setHeader("Content-Type", mimeType);
        res.writeHead(200)
        res.write(data)
        return res.end()
    })
}

//standard web server on port 80 to serve files
var http_server = http.createServer(function (req, res) {
  //see https://nodejs.org/api/http.html#http_class_http_incomingmessage 
  //for the format of q. 
  var q = url.parse(req.url, true)
  console.log("web server passed pathname: " + q.pathname)
  if (q.pathname === "/") {
      q.pathname = "index.html"
  }
  if (q.pathname === "/init_jobs") {
      serve_init_jobs(q, req, res)
  }
  //else if (q.pathname === "/get_refresh") { serve_get_refresh(q, req, res) }
  //else if(q.pathname === "/job_button_click") {
  //	  serve_job_button_click(q, req, res)
  //}
  //else if(q.pathname === "/show_window_button_click") {
  //	  serve_show_window_button_click(q, req, res)
  //} 
  else {
  	  serve_file(q, req, res)
  }
})

http_server.listen(80)
console.log("listening on port 80")

/* orig james N code
function jobs(q, res){
 console.log("serving job list")
    fs.readdir("/srv/samba/share/dde_apps/", function(err, items) {
      if (err) {
        console.log("ERROR:"+err)
        res.writeHead(500, {'Content-Type': 'text/html'})
        return res.end("500 Error")
        }
      res.writeHead(200, {'Content-Type': 'text'})
      for (var i=0; i<items.length; i++) {
        res.write(items[i]+"\n")
        }
      return res.end()
      })
    return
}
*/


wss.on('connection', function(the_ws, req) {
  console.log("\n\nwss got connection: " + the_ws)
  console.log("\nwss SAME AS the_ws : " + (wss === the_ws))
  let browser_socket = the_ws //the_socket used when stdout from job engine comes to the web server process
  the_ws.on('message', function(message) {
    console.log('\n\nwss server on message received: %s', message);
    //the_ws.send("server sent this to browser in response to: " + message)
    let mess_obj = JSON.parse(message)
    console.log("\nwss server on message receieved kind: " + mess_obj.kind)
    if(mess_obj.kind === "keep_alive_click") {
        serve_job_button_click(browser_socket, mess_obj)
    }
    else if(mess_obj.kind === "job_button_click") {
    	serve_job_button_click(browser_socket, mess_obj)
    }
    else if(mess_obj.kind === "show_window_call_callback"){
        serve_show_window_call_callback(browser_socket, mess_obj)
    }
    else {
      console.log("\n\nwss server received invalid message kind: " + mess_obj.kind)
    }
  })
  the_ws.send('websocket connected.\n')
})



//websocket server that connects to Dexter
//socket server to accept websockets from the browser on port 3000
//and forward them out to DexRun as a raw socket
var browser = new ws.Server({ port:3000 })
var bs 
var dexter = new net.Socket()
//don't open the socket yet, because Dexter only allows 1 socket connection
dexter.connected = false //track socket status (doesn't ws do this?)

browser.on('connection', function connection(socket, req) {
  console.log(process.hrtime()[1], " browser connected ", req.connection.Server);
  bs = socket
  socket.on('message', function (data) {
    console.log(process.hrtime()[1], " browser says ", data.toString());
    //Now as a client, open a raw socket to DexRun on localhost
    if (!dexter.connected && !dexter.connecting) { 
      dexter.connect(50000, "127.0.0.1") 
      console.log("dexter connect")
      dexter.on("connect", function () { 
        dexter.connected = true 
        console.log("dexter connected")
        dexter.write(data.toString());
        } )
      dexter.on("data", function (data){
        //console.log(process.hrtime()[1], " dexter says ", data)
        //for(let i = 0; i<8*4; i+=4) {console.log(i, data[i])}
        console.log(process.hrtime()[1], " dexter says ","#"+data[1*4]+" op:"+String.fromCharCode(data[4*4]))
        if (data[5*4]) {console.log("error:"+data[5*4])}
        if (bs) {
            bs.send(data,{ binary: true })
            console.log(process.hrtime()[1], " sent to browser ")
            }
        })
      dexter.on("close", function () { 
        dexter.connected = false 
        console.log("dexter disconnect")
        dexter.removeAllListeners() 
        //or multiple connect/data/close events next time
        } )
      dexter.on("end", function () { 
        dexter.connected = false 
        console.log("dexter ended")
        //dexter.removeAllListeners() 
        dexter.end()
        //or multiple connect/data/close events next time
        } )
      dexter.on("error", function () {
        dexter.connected = false 
        console.log("dexter error")
        if (bs) { bs.send(null,{ binary: true }) }
        dexter.removeAllListeners() 
        dexter.destroy()
        } )
      }
    dexter.write(data.toString());
    });
  socket.on('close', function (data) {
    console.log(process.hrtime()[1], " browser disconnected ");
    bs = null
    dexter.end()
    });
  });


//test to see if we can get a status update from DexRun
//dexter.write("1 1 1 undefined g ;")

