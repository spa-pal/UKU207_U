var palDict = {
	A : "А",
	B : "Б",
	C : "Ц",
	D : "Д",
	E : "Е",
	F : "Ф",
	G : "Г",
	H : "Х",
	I : "И",
	J : "Й",
	K : "К",
	L : "Л",
	M : "М",
	N : "Н",
	O : "О",
	P : "П",
	Q : "Я",
	R : "Р",
	S : "С",
	T : "Т",
	U : "У",
	V : "Ю",
	W : "В",
	XA : "Ш",
	Y : "Ч",
	Z : "З",
	XE : "Ё",
	XC : "Ж",
	XD : "Щ",
	XB : "Ъ",
	XF : "Ы",
	XG : "Ь",
	XH : "Э",
	a : "а",
	b : "б",
	c : "ц",
	d : "д",
	e : "е",
	f : "ф",
	g : "г",
	h : "х",
	i : "и",
	j : "й",
	k : "к",
	l : "л",
	m : "м",
	n : "н",
	o : "о",
	p : "п",
	q : "я",
	r : "р",
	s : "с",
	t : "т",
	u : "у",
	v : "ю",
	w : "в",
	Xa : "ш",
	y : "ч",
	z : "з",
	Xe : "ё",
	Xc : "ж",
	Xd : "щ",
	Xb : "ъ",
	Xf : "ы",
	Xg : "ь",
	Xh : "э",
	Xi : "°",
	Xj : "№"
};

function palDecoder(input) {
	var output = "",i=0;
	while (input[i]){
		if(input[i]=='^'){
			if(input[++i]=='X'){
				output+=palDict[input[i]+input[i+1]];
				i++;
				i++;
			}
			else {
				output += palDict[input[i]];
				i++;
			}
		}
		else {
			output +=input[i++];
		}
	}
	return output;
}

console.log("pal_decoder.js загружен");