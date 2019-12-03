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
  "html": "text/html",
  "mp3":"audio/mpeg",
  "mp4":"video/mp4",
  "jpeg": "image/jpeg",
  "jpg": "image/jpeg",
  "png": "image/png",
  "svg": "image/svg+xml",
  "gif": "image/gif",
  "js": "text/javascript",
  "css": "text/css",
  };

const { spawn } = require('child_process');
var job

function spawn_job_proc(jobfile, out, end) {
    //https://nodejs.org/api/child_process.html
    job = spawn('node'
        , ["core define_and_start_job "+jobfile]
        , {cwd: '/root/Documents/dde', shell: true}
        )
    job.stdout.on('data', function(data) {
        console.log(">"+data)
        out(data.toString().replace('\n',"<BR/>").replace("  ","&nbsp;&nbsp"))
        })
    job.stderr.on('data', function(data) {
        console.log("!"+data)
        out("ERROR: "+data)
        })
    job.on('close', function(code) {
        console.log("closed:"+code)
        out("Finished "+jobfile+". Exit code:"+code)
        return end()
        })
    out("Started "+jobfile+" PID:"+job.pid+"<BR/>")
    return job
    }

//standard web server on port 80 to serve files
const httpd = http.createServer(function (req, res) {
  var q = url.parse(req.url, true)
  if ("/"==q.pathname) 
    q.pathname="index.html"
  if ("/runjob"==q.pathname) {
    var jobfile = "/srv/samba/share/dde_apps/"+q.search.substr(1)
    console.log("Running job "+jobfile+".")
    res.writeHead(200, {'Content-Type': 'text/html'})
    spawn_job_proc(jobfile, function(msg){res.write(msg)}, function(){res.end()})
    return
    }
  if ("/jobs"==q.pathname) {
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
    return //res.end()
    }
  var filename = "/srv/samba/share/www/" + q.pathname
  console.log("serving"+q.pathname)
  fs.readFile(filename, function(err, data) {
    if (err) {
      res.writeHead(404, {'Content-Type': 'text/html'})
      return res.end("404 Not Found")
    }  
    res.setHeader('Access-Control-Allow-Origin', '*');
    let mimeType = mimeTypes[ q.pathname.split(".").pop() ] || "application/octet-stream"
    console.log("Content-Type:",mimeType)
    res.setHeader("Content-Type", mimeType);
    res.writeHead(200)
    res.write(data)
    return res.end()
  });
})
httpd.listen(80)
console.log("listing on port 80")

//handle websocket upgrade request. 
let job_wss = new ws.Server({ noServer: true })
job_wss.child_process = null //reserve a spot for the job process
httpd.on('upgrade', function upgrade(request, socket, head) {
    const pathname = url.parse(request.url).pathname
    if (pathname === '/runjob') { //request to run a job via web socket
        job_wss.handleUpgrade( request, socket, head, function done(ws) {
            job_wss.emit('connection', ws, request) 
            }) //when connected, trigger ws Server's .on('connection') callback
    } else { //what the heck is this for?
        socket.destroy() //reject it.
        }
    })
job_wss.on('connection', function connection(wsocket, req) {
    let jobname = url.parse(req.url, true).search.substr(1)
    let jobfile = "/srv/samba/share/dde_apps/"+jobname
    console.log("job_wss connection "+jobname)
    job_wss.child_process = spawn_job_proc(jobfile
        , function(msg){ //data from job
            console.log("job says:"+msg)
            wsocket.send(msg)
            }
        , function(){ //job ended
            console.log("job ended")
            job_wss.child_process = null
            wsocket.close()
            }
        )
    wsocket.on('message', function incoming(msg) {
//      console.log("\n\n\nbrowser says:"+msg+".\n\n\n")
        let job = job_wss.child_process
        if (job) {
            console.log("\n\n\nbrowser says:"+msg+".\n\n\n")
            job.stdin.write(msg+"\n")
            }
        });
    })

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
