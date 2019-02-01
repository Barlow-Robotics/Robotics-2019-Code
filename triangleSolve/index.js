let tries = 1000000; //How many times you want to run the test
let works = 0; //Triangle Count
for(let i = 0; i < tries; i++){
    let points = [Math.random(),Math.random()]; //Create two random points on line from 0-1
    points.sort(function(a, b){return a - b}); //Sort points from smallest to largest

    let segments = [points[0],points[1]-points[0], 1-points[1]]; //Create 3 segments from those points
    segments.sort(function(a, b){return a - b}); //Sort segments from smallest to largest

    if(segments[0]+segments[1] > segments[2]) //If the sum of the smaller segments are larger than the largest
        works++;                             // segment then increase triangle count by 1
        
}
console.log(`Out of ${tries} tries, there were ${works} triangles ` +
                 `making the chance equal to ${works/tries*100}%`)