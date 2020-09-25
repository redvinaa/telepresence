#! /usr/bin/nodejs

var clargs = process.argv.slice(2);
for (let i=0; i<clargs.length; i++){
	if(clargs[i] == '--help'){
		console.log('agrs: /path/to/pkg:telepresence/ host port');
		return;
	}
}

const express = require('express');
const path = require('path');

const app = express();

app.use(express.json());
app.use(express.static(path.join(clargs[0], 'scripts')));

app.set('view engine', 'pug');
// app.set('views', path.join(__dirname, 'views'));
app.set('views', path.join(clargs[0], 'views'));


app.get('/', (req, res)=>{
	res.render('telepresence');
});

app.listen(clargs[2], clargs[1], ()=>console.log("Server ready on "+clargs[1]+":"+clargs[2]+" (host:port)"));
