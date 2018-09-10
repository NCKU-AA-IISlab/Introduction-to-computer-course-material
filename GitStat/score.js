var tableTotal = document.getElementById("total");
var num_row_total=tableTotal.rows.length;
var num_col_total=tableTotal.rows[0].cells.length;

function calculate(){
	var weight=[0,0.18,0.06,0.36,0.1,0.1];
	var count=0;
	for (var r =1; r < num_row_total ; r++){
		count=0;
		for (var c= 1; c < num_col_total-2;c++){
			a=parseInt(tableTotal.rows[r].cells[c].innerHTML);
			count+=a*weight[c];
		}
		report=parseInt(tableTotal.rows[r].cells[num_col_total-2].children[0].value);
		count+=report*0.2;
		tableTotal.rows[r].cells[num_col_total-1].innerHTML=Math.round(count);
	}
}
